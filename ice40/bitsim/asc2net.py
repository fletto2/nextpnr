#!/usr/bin/env python3
"""asc2net.py X.asc X.net [--pcf vera_module.pcf] [--package sg48]
Decode an iCE40 UP5K .asc to a flat netlist, routing collapsed. Net 0/1 are the
constants, -1 an unconnected output. Net decoding follows icebox_vlog; SPRAM and
MAC16 come from the chipdb .extra_cell blocks."""
import argparse, os, re, sys

SRC = "/opt/oss-fpga/src/icestorm/icebox"
sys.path.insert(0, SRC)
import icebox  # noqa: E402

# unconnected pins that do not read 0
DEFAULT1 = {"CE"}


def read_extra_cells(path):
    cells, cur = [], None
    for line in open(path):
        f = line.split()
        if not f:
            cur = None if line.strip() == "" else cur
            continue
        if f[0] == ".extra_cell":
            cur = None
            if f[-1] in ("MAC16", "SPRAM"):
                loc = tuple(int(v) for v in f[1:-1])
                cur = {"type": f[-1], "loc": loc, "pins": {}, "cfg": {}}
                cells.append(cur)
        elif f[0].startswith("."):
            cur = None
        elif cur is not None:
            x, y = int(f[1]), int(f[2])
            (cur["cfg"] if f[3].startswith("CBIT_") else cur["pins"])[f[0]] = (x, y, f[3])
    return cells


class Decoder:
    def __init__(self, asc, pcf, package):
        self.ic = ic = icebox.iceconfig()
        ic.read_file(asc)
        assert ic.device == "5k", "UP5K only"
        self.out = []
        self.warn = []
        self.nnets = 2
        self.seg2net = {}
        self.labels = {}
        self.pins = {}          # (x, y, z) -> port name
        pinloc = {e[0]: (e[1], e[2], e[3]) for e in ic.pinloc_db(package)}
        if pcf:
            for line in open(pcf):
                f = re.sub(r"#.*", "", line).split()
                if len(f) >= 3 and f[0] == "set_io":
                    self.pins[pinloc[f[2]]] = f[1]
        self.read_io_config()
        self.read_pll()
        self.group()

    def new_net(self):
        self.nnets += 1
        return self.nnets - 1

    def read_io_config(self):
        ic = self.ic
        self.io_type, self.io_neg = {}, set()
        for idx, tile in ic.io_tiles.items():
            tc = icebox.tileconfig(tile)
            t = {0: list("000000"), 1: list("000000")}
            for e in ic.tile_db(idx[0], idx[1]):
                if e[1] == "NegClk" and tc.match(e[0]):
                    self.io_neg.add(idx)
                if e[1].startswith("IOB_") and e[2].startswith("PINTYPE_") and tc.match(e[0]):
                    t[int(e[1][-1])][int(e[2][-1])] = "1"
            for z in (0, 1):
                # text is PIN_TYPE[0]..[5], as icebox prints it
                self.io_type[(idx[0], idx[1], z)] = "".join(t[z])

    def pll_bits(self, info, name, n=None):
        ic = self.ic
        if not hasattr(self, "pll_idx"):
            self.pll_idx = {}
            for e in icebox.iotile_l_db:
                if e[1] == "PLL":
                    m = re.match(r"B(\d+)\[(\d+)\]", e[0][0])
                    self.pll_idx[e[2]] = (int(m[1]), int(m[2]))

        def bit(nm):
            b = info[nm]
            r, c = self.pll_idx[b[2]]
            return ic.tile(b[0], b[1])[r][c]
        if n is None:
            return bit(name)
        return "".join(bit("%s_%d" % (name, i)) for i in range(n - 1, -1, -1))

    def read_pll(self):
        self.plls = []
        self.pll_pads = set()
        for pid in self.ic.pll_list():
            info = icebox.pllinfo_db[pid]
            t = self.pll_bits(info, "PLLTYPE", 3)
            if t != "000":
                assert t == "010", "only SB_PLL40_PAD handled, got PLLTYPE %s" % t
                self.plls.append(info)
                self.pll_pads.add(info["PLLOUT_A"])

    def group(self):
        ic = self.ic
        extra_seg, extra_con = [], []
        self.glb_pads = {}
        for bit in ic.extra_bits:
            e = ic.lookup_extra_bit(bit)
            if e[0] == "padin_glb_netwk":
                pio = ic.padin_pio_db()[int(e[1])]
                s = (pio[0], pio[1], "padin_%d" % pio[2])
                extra_seg.append(s)
                if pio not in self.pll_pads:
                    extra_con.append(((pio[0], pio[1], "io_%d/PAD" % pio[2]), s))
        for cell, name in self.pins.items():
            extra_seg.append((cell[0], cell[1], "io_%d/PAD" % cell[2]))
        groups = sorted(ic.group_segments(extra_connections=extra_con, extra_segments=extra_seg))
        for g in groups:
            n = self.new_net()
            for s in g:
                self.seg2net[s] = n
        self.groups = groups

    def net(self, seg, default=None):
        """net of a segment; default is the const for an input left unrouted (None: fresh net)"""
        n = self.seg2net.get(seg)
        if n is None:
            if default is not None:
                return default
            n = self.new_net()
            self.seg2net[seg] = n
        return n

    def alias(self, a, b):
        self.out.append("alias %d %d" % (a, b))

    def decode_lcs(self):
        ic = self.ic
        nff = nlut = ncarry = 0
        for (x, y), tile in sorted(ic.logic_tiles.items()):
            neg = icebox.get_negclk_bit(tile) == "1"
            casc = icebox.get_carry_cascade_bit(tile) == "1"
            cset = icebox.get_carry_bit(tile) == "1"
            for n in range(8):
                lut = icebox.get_lutff_lut_bits(tile, n)
                seq = icebox.get_lutff_seq_bits(tile, n)
                pfx = "lutff_%d/" % n
                out_used = (x, y, pfx + "out") in self.seg2net or (x, y, pfx + "lout") in self.seg2net
                if "1" not in lut and "1" not in seq and not out_used:
                    continue
                init = int("".join(reversed(lut)), 2)
                i = [self.net((x, y, pfx + "in_%d" % k), 0) for k in range(4)]
                lo = self.net((x, y, pfx + "lout"))
                self.out.append("lut %d %d %d %d %d %04x %d %d %d" % (lo, i[0], i[1], i[2], i[3], init, x, y, n))
                nlut += 1
                if seq[0] == "1":
                    cout = self.net((x, y, pfx + "cout"))
                    if n == 0:
                        cin = self.net((x, y, "carry_in_mux"))
                        if not casc:
                            self.alias(cin, 1 if cset else 0)
                    else:
                        prev = (x, y, "lutff_%d/cout" % (n - 1))
                        if icebox.get_lutff_seq_bits(tile, n - 1)[0] != "1":
                            self.warn.append("carry at %d %d %d fed by LC with carry off" % (x, y, n))
                        cin = self.net(prev, 0)
                    self.out.append("carry %d %d %d %d %d %d %d" % (cout, i[1], i[2], cin, x, y, n))
                    ncarry += 1
                o = self.net((x, y, pfx + "out"))
                if seq[1] == "1":
                    self.out.append("ff %d %d %d %d %d %d %s %s %d %d %d" % (
                        o, lo, self.net((x, y, "lutff_global/clk"), 0),
                        self.net((x, y, "lutff_global/cen"), 1),
                        self.net((x, y, "lutff_global/s_r"), 0),
                        neg, seq[2], seq[3], x, y, n))
                    nff += 1
                else:
                    self.alias(o, lo)
        return nlut, nff, ncarry

    def decode_io(self):
        ic = self.ic
        cells = set(self.pins)
        for s in self.seg2net:
            m = re.match(r"io_(\d+)/(D_IN_\d|D_OUT_\d|OUT_ENB)$", s[2])
            if m and ic.tile_type(s[0], s[1]) == "IO":
                cells.add((s[0], s[1], int(m[1])))
        for c in sorted(cells):
            x, y, z = c
            pfx = "io_%d/" % z
            name = self.pins.get(c, "io_%d_%d_%d" % c)
            typ = self.io_type[c]
            if c in self.pll_pads:
                # input-only reference pad; D_IN_0 here is PLLOUTCORE, tied to clk50 in decode_pll
                self.out.append("refpad %s" % name)
                continue
            pad = self.net((x, y, pfx + "PAD"))
            din0 = self.net((x, y, pfx + "D_IN_0"), -1)
            din1 = self.net((x, y, pfx + "D_IN_1"), -1)
            d0 = self.net((x, y, pfx + "D_OUT_0"), 0)
            d1 = self.net((x, y, pfx + "D_OUT_1"), 0)
            oe = self.net((x, y, pfx + "OUT_ENB"), 1)
            cen = self.net((x, y, "io_global/cen"), 1)
            iclk = self.net((x, y, "io_global/inclk"), 0)
            oclk = self.net((x, y, "io_global/outclk"), 0)
            latch = self.net((x, y, "io_global/latch"), 0)
            neg = 1 if (x, y) in self.io_neg else 0
            self.out.append("io %s %s %d %d %d %d %d %d %d %d %d %d %d %d %d %d" % (
                name, "".join(reversed(typ)), neg, pad, din0, din1, d0, d1, oe, cen, iclk, oclk, latch, x, y, z))

    def decode_pll(self):
        for info in self.plls:
            a = info["PLLOUT_A"]
            glb = self.net((a[0], a[1], "padin_%d" % a[2]))
            core = self.seg2net.get((a[0], a[1], "io_%d/D_IN_0" % a[2]))
            if core is not None:
                self.alias(core, glb)
            lock = self.net(info["LOCK"], -1)
            self.out.append("pll %d %d DIVR=%s DIVF=%s DIVQ=%s OUT=%s" % (
                glb, lock, self.pll_bits(info, "DIVR", 4), self.pll_bits(info, "DIVF", 7),
                self.pll_bits(info, "DIVQ", 3), self.pll_bits(info, "PLLOUT_SELECT_A", 2)))

    def decode_ram(self):
        ic = self.ic
        idx = {}
        t0 = next(iter(ic.ramb_tiles))
        for dy, tag in ((0, "B"), (1, "T")):
            for e in ic.tile_db(t0[0], t0[1] + dy):
                if e[1] == "RamConfig":
                    idx[e[2]] = (tag, e[0])
        nram = 0
        for t in sorted(ic.ramb_tiles):
            cb = icebox.tileconfig(ic.tile(t[0], t[1]))
            ct = icebox.tileconfig(ic.tile(t[0], t[1] + 1))

            def cfg(name):
                return (cb if idx[name][0] == "B" else ct).match(idx[name][1])

            def wire(name, msb=None, default=0):
                r = []
                for i in (range(msb + 1) if msb is not None else [None]):
                    n = "ram/%s" % name + ("_%d" % i if i is not None else "")
                    v = self.seg2net.get((t[0], t[1], n), self.seg2net.get((t[0], t[1] + 1, n), default))
                    r.append(v)
                return r
            if not cfg("PowerUp"):
                continue
            negw = icebox.get_negclk_bit(ic.tile(t[0], t[1])) == "1"
            negr = icebox.get_negclk_bit(ic.tile(t[0], t[1] + 1)) == "1"
            rm = int(cfg("CBIT_2")) + 2 * int(cfg("CBIT_3"))
            wm = int(cfg("CBIT_0")) + 2 * int(cfg("CBIT_1"))
            data = ic.ram_data.get(t, ["0" * 64] * 16)
            f = [t[0], t[1], rm, wm, int(negr), int(negw)]
            f += wire("RCLK") + wire("RCLKE", default=1) + wire("RE")
            f += wire("WCLK") + wire("WCLKE", default=1) + wire("WE")
            f += wire("RADDR", 10) + wire("WADDR", 10) + wire("MASK", 15) + wire("WDATA", 15)
            f += wire("RDATA", 15, -1)
            self.out.append("ram " + " ".join(map(str, f)) + " " + "".join(data))
            nram += 1
        return nram

    def cbit(self, spec):
        x, y, name = spec
        for e in self.ic.tile_db(x, y):
            if e[1] == "IpConfig" and e[2] == name:
                return int(icebox.tileconfig(self.ic.tile(x, y)).match(e[0]))
        raise KeyError(spec)

    def decode_ip(self):
        cells = read_extra_cells(os.path.join(SRC, "chipdb-5k.txt"))
        n = {"MAC16": 0, "SPRAM": 0}
        for c in cells:
            pins = c["pins"]
            used = any(p in self.seg2net for p in pins.values())
            cfg = {k: self.cbit(v) for k, v in c["cfg"].items()}
            if c["type"] == "SPRAM":
                if not cfg["SPRAM_EN"]:
                    assert not used
                    continue
                vals = [("%d" % self.net(pins[k], -1 if k.startswith("DATAOUT") else 0)) for k in spram_pins()]
                self.out.append("spram %d %d %d " % c["loc"] + " ".join(vals))
            else:
                if not used:
                    continue
                vals = [("%d" % self.net(pins[k], -1 if k.startswith("O_") or k == "CO" else
                                         (1 if k in DEFAULT1 else 0))) for k in mac_pins()]
                conf = " ".join("%s=%d" % kv for kv in sorted(cfg.items()))
                # a MAC16 has no NEG_TRIGGER CBIT; the tile holding its clock has the LC NegClk bit
                clk = pins["CLK"]
                neg = icebox.get_negclk_bit(self.ic.tile(clk[0], clk[1]))
                self.out.append("mac %d %d %d NEG_TRIGGER=%s " % (c["loc"] + (neg,)) + conf + " : " + " ".join(vals))
            n[c["type"]] += 1
        return n

    def run(self):
        counts = self.decode_lcs()
        self.decode_pll()
        self.decode_io()
        nram = self.decode_ram()
        ip = self.decode_ip()
        return counts, nram, ip


def spram_pins():
    return (["CLOCK", "CHIPSELECT", "WREN", "STANDBY", "SLEEP", "POWEROFF"] +
            ["MASKWREN_%d" % i for i in range(4)] + ["ADDRESS_%d" % i for i in range(14)] +
            ["DATAIN_%d" % i for i in range(16)] + ["DATAOUT_%d" % i for i in range(16)])


def mac_pins():
    return (["CLK", "CE", "AHOLD", "BHOLD", "CHOLD", "DHOLD", "IRSTTOP", "IRSTBOT", "ORSTTOP", "ORSTBOT",
             "OLOADTOP", "OLOADBOT", "ADDSUBTOP", "ADDSUBBOT", "OHOLDTOP", "OHOLDBOT", "CI"] +
            ["A_%d" % i for i in range(16)] + ["B_%d" % i for i in range(16)] +
            ["C_%d" % i for i in range(16)] + ["D_%d" % i for i in range(16)] +
            ["O_%d" % i for i in range(32)] + ["CO"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asc")
    ap.add_argument("net")
    ap.add_argument("--pcf")
    ap.add_argument("--package", default="sg48")
    a = ap.parse_args()
    d = Decoder(a.asc, a.pcf, a.package)
    (nlut, nff, ncarry), nram, ip = d.run()
    with open(a.net, "w") as f:
        f.write("bitsim 1\nnets %d\n" % d.nnets)
        f.write("\n".join(d.out) + "\n")
    print("%d nets, %d LC luts, %d FF, %d carry, %d RAM, %d MAC16, %d SPRAM" % (
        d.nnets, nlut, nff, ncarry, nram, ip["MAC16"], ip["SPRAM"]))
    for w in d.warn:
        print("warning:", w)


if __name__ == "__main__":
    main()
