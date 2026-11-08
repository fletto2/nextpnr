#!/usr/bin/env python3
"""tgen.py X.net X.tv OUT.v [--corner 0|1|2] [--scale F] [--mem-order 3,2,4,1]: timed simulation of an image.

Function comes from X.net, the same netlist the fast engine runs. X.tv (icetime -o) has no function left in it,
no RAM contents, MAC setup or FF set/reset, so it only gives delays: per cell input pin, driver to pin through
every routing stage, joined on the cell's position. Zero delay cells get a delay on each input instead:
route + arc to the output for LUTs, route + setup for FFs and memories, so a late D is simply not seen.
Run the result with bench args +ev= +out= +clk50= +mem= +stop= +edge= +acc=. Transport delays, rise and fall kept
separate along a route; a pulse whose quick edge overtakes its slow one still ends on the input's value.
Power up like the chip: everything settles with no delay, async sets hold, no clock edge before 10 ns."""
import argparse, os, re, sys
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen, tmnet

NO = -1


def table(corner):
    a, su = {}, defaultdict(dict)
    cell = None
    v = lambda s: float(s.split(":")[corner].replace("*", "0"))
    for line in open(tmnet.TIMINGS):
        w = line.split()
        if not w:
            continue
        if w[0] == "CELL":
            cell = w[1]
        elif w[0] == "IOPATH":
            k = (cell, w[1].split(":")[-1], w[2])
            r, f = v(w[3]), v(w[4])
            o = a.get(k, (0, 0))
            a[k] = (max(o[0], r), max(o[1], f))
        elif w[0] == "SETUP":
            e, pin = w[1].split(":")
            d = su[cell].setdefault(pin, [0, 0])
            d[e == "negedge"] = max(d[e == "negedge"], v(w[3]))
    return a, su


class Routes:
    def __init__(self, tv, corner):
        self.arc, self.setup = table(corner)
        insts, assigns = tmnet.parse_tv(tv)
        self.rep = {}
        for x, y in assigns:
            self.rep[self.root(x)] = self.root(y)
        self.inst, self.drv, self.src = {}, {}, {}
        for ty, name, par, pins in insts:
            flat = {}
            for k, val in pins.items():
                val = val.strip()
                if val.startswith("{"):
                    bits = [b.strip() for b in val[1:-1].split(",")]
                    for i, b in enumerate(reversed(bits)):
                        flat["%s[%d]" % (k, i)] = self.root(b)
                else:
                    flat[k] = self.root(val)
            self.inst[name] = (ty, flat)
            io = {"ICE_GB": ("USERSIGNALTOGLOBALBUFFER", "GLOBALBUFFEROUTPUT"),
                  "ICE_CARRY_IN_MUX": ("carryinitin", "carryinitout")}.get(ty, ("I", "O"))
            if io[0] in flat and io[1] in flat:
                r, f = self.arc.get((ty, io[0], io[1]), (0, 0))
                self.drv[flat[io[1]]] = (flat[io[0]], r, f)
            elif ty == "LogicCell40":
                for p in ("lcout", "ltout", "carryout"):
                    if p in flat:
                        self.src[flat[p]] = (name, p)

    def root(self, n):
        m = re.search(r"glb_netwk_(\d)", n)
        if m:
            return "GLB" + m[1]
        while n in self.rep:
            n = self.rep[n]
        return n

    def walk(self, inst, pin):
        """(rise, fall, source) from the driving cell to this pin; None when the netlist has no such pin"""
        t = self.inst.get(inst)
        if not t or pin not in t[1]:
            return None
        n, r, f = t[1][pin], 0.0, 0.0
        seen = 0
        while n in self.drv and seen < 400:
            n, a, b = self.drv[n]
            r, f, seen = r + a, f + b, seen + 1
        return r, f, self.src.get(n)


MODELS = r"""
`timescale 1ps/1ps
module dl #(parameter R = 0, F = 0) (input a, output reg y);
	initial begin y = 0; #1 y = a === 1'b1; end // an input set at time 0 can beat the always block
	// the chip has long settled when it wakes: no delays until the first picoseconds are over
	always @(a) if ($time < 5) y <= a === 1'b1;
		else begin
			if (a === 1'b1) y <= #R 1'b1; else y <= #F 1'b0;
			// a short pulse's quicker edge can pass the slower one, the last word has to be the input's
			y <= #(R > F ? R : F) a === 1'b1;
		end
endmodule
module lut4 #(parameter [15:0] T = 0) (input a, b, c, d, output y);
	assign y = T[{d === 1'b1, c === 1'b1, b === 1'b1, a === 1'b1}];
endmodule
module ffc #(parameter NEG = 0, SET = 0, ASYNC = 0, CQ = 0, SQ = 0) (input clk, ce, sr, d, output reg q);
	// the nets settle from x at time 0, a two-valued device has no such thing
	wire c = (clk === 1'b1) ^ NEG[0], dd = d === 1'b1, ee = ce === 1'b1, ss = sr === 1'b1;
	// silicon wakes with its clock nets already at level: no edge before the first real one
	reg arm = 0;
	initial begin q = 0; #10000 arm = 1; end
	generate if (ASYNC) begin
		// a set held from power up has no edge to see
		initial #3 if (ss) q = SET[0];
		always @(posedge c or posedge ss)
			if (ss) q <= #SQ SET[0]; else if (ee && arm) q <= #CQ dd;
	end else begin
		always @(posedge c)
			if (ee && arm) q <= #CQ (ss ? SET[0] : dd);
	end endgenerate
endmodule
module spr #(parameter Q = 0) (input clk, cs, wren, standby, sleep, pwr, input [13:0] a, input [15:0] d,
	input [3:0] mw, output reg [15:0] o);
	reg [15:0] mem [0:16383];
	integer i;
	initial begin o = 0; for (i = 0; i < 16384; i = i + 1) mem[i] = 0; end
	always @(posedge clk)
		if (sleep || !pwr) o <= #Q 0;
		else if (!standby && cs) begin
			if (!wren) o <= #Q mem[a];
			else begin
				if (mw[0]) mem[a][3:0] <= d[3:0];
				if (mw[1]) mem[a][7:4] <= d[7:4];
				if (mw[2]) mem[a][11:8] <= d[11:8];
				if (mw[3]) mem[a][15:12] <= d[15:12];
			end
		end
endmodule
"""


class TGen:
    def __init__(self, a):
        self.n = gen.Net(a.net)
        self.rt = Routes(a.tv, a.corner)
        self.scale = a.scale
        self.o = []
        self.dls = {}
        self.lt = {}
        self.miss = defaultdict(int)
        self.pos = {}       # (x, y, z) -> lut
        self.carry = []
        self.ffs = []
        for line in open(a.net):
            f = line.split()
            if f[0] == "lut":
                self.pos[tuple(map(int, f[7:10]))] = (int(f[1]), [int(v) for v in f[2:6]], f[6])
            elif f[0] == "carry":
                self.carry.append((int(f[1]), int(f[2]), int(f[3]), int(f[4]), tuple(map(int, f[5:8]))))
            elif f[0] == "ff":
                self.ffs.append((list(map(int, f[1:9])), tuple(map(int, f[9:12]))))
        clks = sorted(w[0] for w in (self.rt.walk("lc40_%d_%d_%d" % p, "clk") for _, p in self.ffs) if w)
        self.clk_default = clks[len(clks) // 2]

    def ps(self, x):
        return int(round(x * self.scale))

    def net(self, a):
        a = self.n.find(a) if a >= 0 else a
        if a == 0 or a == NO:
            return "1'b0"
        if a == 1:
            return "1'b1"
        return "n%d" % a

    def delayed(self, s, r, f):
        r, f = self.ps(r), self.ps(f)
        if s.startswith("1'b") or not (r or f):
            return s
        k = (s, r, f)
        if k not in self.dls:
            w = "w%d" % len(self.dls)
            self.dls[k] = w
            self.o.append("wire %s; dl #(%d,%d) u%s(%s,%s);" % (w, r, f, w, s, w))
        return self.dls[k]

    def drive(self, src, r, f, a):
        """net a driven from src after a fixed cell delay"""
        r, f = self.ps(r), self.ps(f)
        if not (r or f):
            return "assign %s = %s === 1'b1;" % (self.net(a), src)
        self.ndrive = getattr(self, "ndrive", 0) + 1
        return "dl #(%d,%d) d%d(%s,%s);" % (r, f, self.ndrive, src, self.net(a))

    def pin(self, inst, pin, a, er=0.0, ef=0.0):
        """net a as seen at this pin, plus er/ef on top of the route"""
        s = self.net(a)
        if s.startswith("1'b"):
            return s
        w = self.rt.walk(inst, pin)
        if w is None:
            self.miss[self.rt.inst.get(inst, ("?",))[0] + "." + re.sub(r"\[.*", "", pin)] += 1
            return self.delayed(s, er, ef)
        r, f, src = w
        if src and src[1] == "ltout":
            m = re.match(r"lc40_(\d+)_(\d+)_(\d+)", src[0])
            p = tuple(map(int, m.groups()))
            s = self.lt.setdefault(p, "lt_%d_%d_%d" % p)
        return self.delayed(s, r + er, f + ef)

    def clkpin(self, inst, pin, a):
        """at scale 0 a derived clock still has to come after the data, the delta the zero delay engines give it"""
        w = self.pin(inst, pin, a)
        if self.scale or w.startswith("1'b") or self.n.find(a) == self.n.find(self.n.plls[0][0]):
            return w
        k = ("tiny", w)
        if k not in self.dls:
            self.dls[k] = "t%d" % len(self.dls)
            self.o.append("wire %s; dl #(1,1) u%s(%s,%s);" % (self.dls[k], self.dls[k], w, self.dls[k]))
        return self.dls[k]

    def lut(self, p, ins, init, y, mode):
        """mode: lcout, ltout or setup, what the input delay carries beyond the route"""
        inst = "lc40_%d_%d_%d" % p
        ws = []
        for i, a in enumerate(ins):
            pn = "in%d" % i
            if mode == "setup":
                su = self.rt.setup["LogicCell40"].get(pn, [0, 0])
                er, ef = su[0], su[1]
            else:
                er, ef = self.rt.arc.get(("LogicCell40", pn, mode), (0, 0))
            ws.append(self.pin(inst, pn, a, er, ef))
        self.o.append("lut4 #(16'h%s) l_%s(%s,%s);" % (init, y, ",".join(ws), y))

    def run(self, a):
        n, o, rt = self.n, self.o, self.rt
        arc = rt.arc
        ffpos = {p for _, p in self.ffs}
        for p, (out, ins, init) in self.pos.items():
            self.lut(p, ins, init, self.net(out), "setup" if p in ffpos else "lcout")
        for out, i1, i2, ci, p in self.carry:
            inst = "lc40_%d_%d_%d" % p
            x = self.pin(inst, "in1", i1, *arc.get(("LogicCell40", "in1", "carryout"), (0, 0)))
            y = self.pin(inst, "in2", i2, *arc.get(("LogicCell40", "in2", "carryout"), (0, 0)))
            z = self.pin(inst, "carryin", ci, *arc.get(("LogicCell40", "carryin", "carryout"), (0, 0)))
            o.append("lut4 #(16'hE8E8) c_%d(%s,%s,%s,1'b0,%s);" % (out, x, y, z, self.net(out)))
        cq = arc[("LogicCell40", "clk", "lcout")][0]
        sq = max(arc[("LogicCell40", "sr", "lcout")])
        for (q, d, clk, cen, sr, neg, setv, asyn), p in self.ffs:
            inst = "lc40_%d_%d_%d" % p
            sc = rt.setup["LogicCell40"]
            o.append("ffc #(%d,%d,%d,%d,%d) f_%d(%s,%s,%s,%s,%s);" % (
                neg, setv, asyn, self.ps(cq), self.ps(sq), q, self.clkpin(inst, "clk", clk),
                self.pin(inst, "ce", cen, *sc.get("ce", (0, 0))), self.pin(inst, "sr", sr, *sc.get("sr", (0, 0))),
                self.net(d), self.net(q)))
        for i, r in enumerate(n.rams):
            inst = "ram_%d_%d" % (r["x"], r["y"])
            if r["nr"] or r["nw"]:
                sys.exit("negative clock RAM not handled")
            su = rt.setup["SB_RAM40_4K"]

            def rp(pn, net):
                b = su.get(pn, [0, 0])
                return self.pin(inst, pn, net, b[0], b[1])
            vec = lambda nm, nets: "{" + ",".join(rp("%s[%d]" % (nm, j), nets[j]) for j in reversed(range(len(nets)))) + "}"
            init = r["init"]
            par = ",".join(".INIT_%X(256'h%s)" % (b, init[64 * b:64 * b + 64]) for b in range(16))
            o.append(self.ram_inst(i, r, inst, rp, vec, par))
            q = arc[("SB_RAM40_4K", "RCLK", "RDATA[0]")][0]
            for j, a2 in enumerate(r["rdata"]):
                if a2 != NO:
                    o.append(self.drive("rd%d[%d]" % (i, j), q, q, a2))
        sp = [l.split() for l in open(a.net) if l.startswith("spram ")]
        for i, s in enumerate(n.sprams):
            s["name"] = "SPRAM_%s_%s_%s" % tuple(sp[i][1:4])
            self.spram(i, s)
        for i, m in enumerate(n.macs):
            self.mac(i, m)
        self.ios(a)

    def ram_inst(self, i, r, inst, rp, vec, par):
        return ("wire [15:0] rd%d; SB_RAM40_4K #(.READ_MODE(%d),.WRITE_MODE(%d),%s) r%d(.RDATA(rd%d),.RCLK(%s),.RCLKE(%s),.RE(%s),.RADDR(%s),"
                ".WCLK(%s),.WCLKE(%s),.WE(%s),.WADDR(%s),.MASK(%s),.WDATA(%s));" % (
                    i, r["rm"], r["wm"], par, i, i, self.clkpin(inst, "RCLK", r["rclk"]), rp("RCLKE", r["rclke"]), rp("RE", r["re"]),
                    vec("RADDR", r["raddr"]), self.clkpin(inst, "WCLK", r["wclk"]), rp("WCLKE", r["wclke"]), rp("WE", r["we"]),
                    vec("WADDR", r["waddr"]), vec("MASK", r["mask"]), vec("WDATA", r["wdata"])))

    def clocked(self, net):
        if not self.scale:
            return self.clkpin("", "", net)
        return self.delayed(self.net(net), self.clk_default, self.clk_default)

    def spram(self, i, s):
        p = s["p"]
        name = s["name"]
        self.o.append("// spram idx %d is %s" % (s["idx"], name))
        pn = lambda k: self.pin(name, k, p[k])
        bus = lambda nm, w: "{" + ",".join(self.pin(name, "%s[%d]" % (nm, j), p["%s_%d" % (nm, j)]) for j in reversed(range(w))) + "}"
        q = self.rt.arc[("SB_SPRAM256KA", "CLOCK", "DATAOUT[0]")][0]
        self.o.append("wire [15:0] so%d; spr #(%d) sp%d(%s,%s,%s,%s,%s,%s,%s,%s,%s,so%d);" % (
            i, self.ps(q), s["idx"], self.clocked(p["CLOCK"]), pn("CHIPSELECT"), pn("WREN"), pn("STANDBY"), pn("SLEEP"), pn("POWEROFF"),
            bus("ADDRESS", 14), bus("DATAIN", 16), bus("MASKWREN", 4), i))
        for j in range(16):
            a = p["DATAOUT_%d" % j]
            if a != NO:
                self.o.append("assign %s = so%d[%d];" % (self.net(a), i, j))

    def mac(self, i, m):
        p, c = m["p"], m["cfg"]
        name = "MAC16_%d_%d_0" % (m["x"], m["y"])
        two = lambda k: c[k + "_0"] | c[k + "_1"] << 1
        par = [".%s(%d)" % (k, c[k]) for k in ("A_REG", "B_REG", "C_REG", "D_REG", "TOP_8x8_MULT_REG", "BOT_8x8_MULT_REG",
               "PIPELINE_16x16_MULT_REG1", "PIPELINE_16x16_MULT_REG2", "TOPADDSUB_UPPERINPUT", "BOTADDSUB_UPPERINPUT", "MODE_8x8",
               "A_SIGNED", "B_SIGNED")]
        par += [".%s(%d)" % (k, two(k)) for k in ("TOPOUTPUT_SELECT", "TOPADDSUB_LOWERINPUT", "TOPADDSUB_CARRYSELECT",
                "BOTOUTPUT_SELECT", "BOTADDSUB_LOWERINPUT", "BOTADDSUB_CARRYSELECT")]
        one = lambda k: self.pin(name, k, p[k])
        bus = lambda nm: "{" + ",".join(self.pin(name, "%s[%d]" % (nm, j), p["%s_%d" % (nm, j)]) for j in reversed(range(16))) + "}"
        ports = [".CLK(%s)" % self.clocked(p["CLK"]), ".CE(%s)" % self.net(p["CE"])]
        for k in ("AHOLD", "BHOLD", "CHOLD", "DHOLD", "IRSTTOP", "IRSTBOT", "ORSTTOP", "ORSTBOT", "OLOADTOP", "OLOADBOT",
                  "ADDSUBTOP", "ADDSUBBOT", "OHOLDTOP", "OHOLDBOT", "CI"):
            ports.append(".%s(%s)" % (k, one(k)))
        for k in "ABCD":
            ports.append(".%s(%s)" % (k, bus(k)))
        d = max(max(v) for k, v in self.rt.arc.items() if k[0] == "SB_MAC16_MUL_U_16X16_BYPASS" and k[2].startswith("O["))
        self.o.append("initial begin %s end" % " ".join("m%d.%s = 0;" % (i, r) for r in
                      ("rA", "rB", "rC", "rD", "rF", "rG", "rH", "rJ", "rK", "rQ", "rS")))
        self.o.append("wire [31:0] mo%d; wire mc%d; SB_MAC16 #(%s) m%d(%s,.ACCUMCI(1'b0),.SIGNEXTIN(1'b0),.O(mo%d),.CO(mc%d));" % (
            i, i, ",".join(par), i, ",".join(ports), i, i))
        for j in range(32):
            a = p["O_%d" % j]
            if a != NO:
                self.o.append(self.drive("mo%d[%d]" % (i, j), d, d, a))
        if p.get("CO", NO) != NO:
            self.o.append(self.drive("mc%d" % i, d, d, p["CO"]))

    def ios(self, a):
        n, o, arc = self.n, self.o, self.rt.arc
        pad_in = arc[("IO_PAD", "PACKAGEPIN", "DOUT")]
        pin_in = arc[("PRE_IO", "PADIN", "DIN0")]
        pad_o = arc[("IO_PAD", "DIN", "PACKAGEPIN")]
        pre_o = arc[("PRE_IO", "DOUT0", "PADOUT")]
        pre_ck = arc[("PRE_IO", "OUTPUTCLK", "PADOUT")]
        pre_oe = arc[("PRE_IO", "OUTPUTENABLE", "PADOEN")]
        self.pads = []
        for line in open(a.net):
            f = line.split()
            if f[0] != "io":
                continue
            io = next(x for x in n.ios if x["name"] == f[1])
            inst = "pre_io_%s_%s_%s" % tuple(f[-3:])
            t = io["typ"]
            din, dout, oen = t & 3, t >> 2 & 3, t >> 4 & 3
            nm = "p_" + re.sub(r"\W", "_", f[1])
            self.pads.append((f[1], nm))
            o.append("wire %s;" % nm)
            if io["din0"] != NO:
                o.append(self.drive(nm, pad_in[0] + pin_in[0], pad_in[1] + pin_in[1], io["din0"]))
            if oen == 0:
                continue
            if dout in (1, 3):
                q = nm + "_q"
                o.append("wire %s; ffc #(%d,0,0,%d,0) oq_%s(%s,%s,1'b0,%s,%s);" % (
                    q, io["neg"], self.ps(pre_ck[0]), nm, self.clkpin(inst, "OUTPUTCLK", io["oclk"]), self.net(io["cen"]),
                    self.pin(inst, "DOUT0", io["d0"]) if dout == 1 else "~" + self.pin(inst, "DOUT0", io["d0"]), q))
                val = self.delayed(q, pad_o[0], pad_o[1])
            else:
                val = self.pin(inst, "DOUT0", io["d0"], pre_o[0] + pad_o[0], pre_o[1] + pad_o[1])
            if oen == 1:
                o.append("assign %s = %s;" % (nm, val))
            else:
                oe = self.pin(inst, "OUTPUTENABLE", io["oe"], pre_oe[0] + pad_o[0], pre_oe[1] + pad_o[1])
                o.append("assign %s = %s ? %s : 1'bz;" % (nm, oe, val))

    def write(self, a):
        n = self.n
        wires = set(re.findall(r"\bn\d+\b", "\n".join(self.o))) | set(self.lt.values())
        for p, w in self.lt.items():
            out, ins, init = self.pos[p]
            self.lut(p, ins, init, w, "ltout")
        wires |= set(re.findall(r"\bn\d+\b", "\n".join(self.o)))
        g, l = n.plls[0]
        cols = ["extbus_irq_n"] + ["vga_%s[%d]" % (c, b) for c in "rgb" for b in (3, 2, 1, 0)] + [
            "vga_hsync", "vga_vsync", "spi_sck", "spi_mosi", "spi_ssel_n_sd", "audio_lrck", "audio_bck", "audio_data"] + [
            "extbus_d[%d]" % b for b in range(7, -1, -1)]
        pn = dict(self.pads)
        with open(a.out, "w") as f:
            f.write(MODELS)
            f.write("module tb;\n")
            f.write("".join("tri0 %s;\n" % w for w in sorted(wires)))
            f.write("\n".join(self.o) + "\n")
            f.write("reg clk50 = 0; assign %s = clk50; assign %s = 1'b1;\n" % (self.net(g), self.net(l)))
            f.write(BENCH % dict(
                cs=pn["extbus_cs_n"], rd=pn["extbus_rd_n"], wr=pn["extbus_wr_n"], miso=pn["spi_miso"],
                a=",".join(pn["extbus_a[%d]" % b] for b in range(4, -1, -1)),
                drv="".join("assign %s = drive ? bd[%d] : 1'bz;\n" % (pn["extbus_d[%d]" % b], b) for b in range(8)),
                cols=",".join(pn[c] for c in cols), ncol=len(cols),
                mem="".join('$readmemh({mem,"%d"}, sp%s.mem);\n' % (k, z) for k, z in enumerate(a.mem_order.split(",")))))
        print("%d delays, %d ltout, missing pins: %s" % (len(self.dls), len(self.lt), dict(self.miss)))


BENCH = r"""
time tend = 0; reg pend = 0, have = 0; reg [7:0] dend, lastd; integer rbad = 0, rtot = 0, lost = 0;
reg cs = 1, rd = 1, wr = 1, drive = 0, miso = 0; reg [4:0] ba = 0; reg [7:0] bd = 0;
assign %(cs)s = cs; assign %(rd)s = rd; assign %(wr)s = wr; assign %(miso)s = miso;
assign {%(a)s} = ba;
%(drv)s
wire [%(ncol)d-1:0] got = {%(cols)s};
reg [8*600:1] evp, outp, mem; integer fe, fo, r, stop, off, ck, maxp = 20;
integer bad [0:%(ncol)d-1]; time first [0:%(ncol)d-1]; integer total = 0, nbad = 0, k; reg [%(ncol)d*8-1:0] sline;
time te, ts, tx; reg [2:0] s3; reg [4:0] e_a; reg [7:0] e_d; reg e_dr, e_mi;
initial begin
	if (!$value$plusargs("ev=%%s", evp) || !$value$plusargs("out=%%s", outp) || !$value$plusargs("clk50=%%d", ck)) $finish;
	if (!$value$plusargs("stop=%%d", stop)) stop = 2000000000;
	if (!$value$plusargs("maxp=%%d", maxp)) maxp = 20;
	if (!$value$plusargs("edge=%%d", off)) off = 17000;
	if ($value$plusargs("mem=%%s", mem)) begin
%(mem)s	end
	for (k = 0; k < %(ncol)d; k = k + 1) bad[k] = 0;
	#(ck) forever begin clk50 = ~clk50; #10000; end
end
initial begin : events
	#1; fe = $fopen(evp, "r");
	while ($fscanf(fe, "%%d %%b %%b %%b %%b %%b", te, s3, e_a, e_d, e_dr, e_mi) == 6) begin
		if (te > $time) #(te - $time);
		// the CPU takes read data as the strobe ends: keep what the pads show at that instant
		if (!cs && !rd && !drive && (s3[2] | s3[1])) begin
			if (pend) lost = lost + 1;
			tend = $time; pend = 1;
			for (k = 0; k < 8; k = k + 1) dend[7 - k] = got[%(ncol)d - 22 - k] === 1'b1;
		end
		{cs, rd, wr} = s3; ba = e_a; bd = e_d; drive = e_dr; miso = e_mi;
	end
end
reg [30*8:1] smp; reg [8:0] ch; integer ci;
initial begin : samples
	#1; fo = $fopen(outp, "r");
	while ($fscanf(fo, "%%d %%s", ts, smp) == 2) begin
		// a reference sample shows the state after the last clk50 rise; the traces put it at different
		// phases, so the timed sample goes a fixed time after that rise, past every clock to pad delay
		tx = ts < ck ? ts + off : ck + (ts - ck) / 20000 * 20000 + off;
		if (tx > stop) begin report; $finish; end
		if (tx > $time) #(tx - $time);
		// a strobe ended before this sample: its data against the last driven value the reference had
		if (pend && ts > tend) begin
			pend = 0;
			if (have) begin
				rtot = rtot + 1;
				if (dend !== lastd) begin
					rbad = rbad + 1; nbad = nbad + 1;
					if (nbad <= maxp) $display("%%0t read at strobe end ref %%b sim %%b", tend, lastd, dend);
				end
			end
			have = 0;
		end
		ch = (smp >> 64) & 255;
		if (ch == "0" || ch == "1") begin
			for (k = 0; k < 8; k = k + 1) lastd[7 - k] = ((smp >> (8 * (8 - k))) & 255) == "1";
			have = 1;
		end
		for (k = 0; k < %(ncol)d; k = k + 1) begin
			ch = smp >> (8 * (29 - k));
			ch = ch & 255;
			// read data is checked only where the CPU takes it, above
			if ((ch == "0" || ch == "1") && k < 21) begin
				total = total + 1;
				if (got[%(ncol)d - 1 - k] !== (ch == "1")) begin
					if (!bad[k]) first[k] = $time;
					bad[k] = bad[k] + 1; nbad = nbad + 1;
					if (nbad <= maxp) $display("%%0t col %%0d ref %%c sim %%b", $time, k, ch, got[%(ncol)d - 1 - k]);
				end
			end
		end
	end
	report; $finish;
end
task report; begin
	$write("to %%0t ps: compared %%0d, reads %%0d (overlapped %%0d), mismatches %%0d, reads %%0d, by column:", $time, total, rtot, lost, nbad, rbad);
	for (k = 0; k < %(ncol)d; k = k + 1) if (bad[k]) $write(" %%0d:%%0d@%%0d", k, bad[k], first[k] / 1000);
	$write("\n");
end endtask
endmodule
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("net")
    ap.add_argument("tv")
    ap.add_argument("out")
    ap.add_argument("--corner", type=int, default=1)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--mem-order", default="3,2,4,1")
    a = ap.parse_args()
    g = TGen(a)
    g.run(a)
    g.write(a)


if __name__ == "__main__":
    main()
