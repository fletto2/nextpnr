#!/usr/bin/env python3
"""tmnet.py X.asc X.tv [--corner 0|1|2] [--pads REGEX]: routed delays of every pad -> FF path.

X.tv is `icetime -d up5k -P sg48 -p x.pcf -o X.tv X.asc`, the image as timing primitives, one per
routing stage. Delays are timings_up5k.txt's, corner 0 min, 1 typ, 2 max, worst of rise and fall.
Prints one line per pad and sink: pad pin sink min_ps max_ps global clk_ps setup_ps. clk_ps runs from the
sink's global to its clock pin; a pad that drives a global shows as a sink with pin clk."""
import argparse, os, re, sys
from collections import defaultdict

TIMINGS = "/opt/oss-fpga/share/icebox/timings_up5k.txt"


def load_timings(path, corner):
    """cell -> [(from, to, lo, hi)], setup[cell][pin]"""
    arcs, setup, cell = defaultdict(list), defaultdict(dict), None
    for line in open(path):
        w = line.split()
        if not w:
            continue
        if w[0] == "CELL":
            cell = w[1]
        elif w[0] == "IOPATH":
            d = [float(x.split(":")[corner].replace("*", "0")) for x in w[3:5]]
            d = [x for x in d if x] or [0.0]
            arcs[cell].append((w[1], w[2], min(d), max(d)))
        elif w[0] == "SETUP":
            pin = w[1].split(":")[1]
            setup[cell][pin] = max(setup[cell].get(pin, 0), float(w[3].split(":")[corner].replace("*", "0")))
    return arcs, setup


def parse_tv(path):
    """instances as (type, name, params, {pin: net}); assigns as pairs"""
    insts, assigns = [], []
    txt = open(path).read()
    for m in re.finditer(r"^  assign (\S+) = (\S+);", txt, re.M):
        assigns.append((m[1], m[2]))
    for m in re.finditer(r"^  (\w+)(?: #\((.*?)\n  \))? (\S+) \((.*?)\);$", txt, re.M | re.S):
        pins = dict(re.findall(r"\.(\w+)\(([^)]*)\)", m[4]))
        insts.append((m[1], m[3], m[2] or "", {k: v for k, v in pins.items() if v}))
    return insts, assigns


def load_syms(path):
    sym = {}
    for line in open(path):
        if line.startswith(".sym "):
            w = line.split()
            sym.setdefault("net_" + w[1], w[2])
    return sym


class Graph:
    def __init__(self, tv, corner):
        arcs, self.setup = load_timings(TIMINGS, corner)
        insts, assigns = parse_tv(tv)
        self.rep = {}
        for a, b in assigns:
            self.rep[self.root(a)] = self.root(b)
        self.fan = defaultdict(list)      # net -> [(net, lo, hi)]
        self.sinks = defaultdict(list)    # net -> [(inst type, name, pin)]
        self.ff = {}                      # lc name -> pins
        self.pads = {}
        for ty, name, par, pins in insts:
            pins = {k: self.root(v) for k, v in pins.items()}
            dff = ty == "LogicCell40" and "SEQ_MODE(4'b1" in par
            if ty == "LogicCell40" and dff:
                self.ff[name] = pins
            if ty == "IO_PAD":
                self.pads[pins["PACKAGEPIN"]] = pins["PACKAGEPIN"]
            for f, t, lo, hi in arcs.get(ty, ()):
                if ":" in f:
                    continue      # clock to out starts a new path
                if ty == "LogicCell40" and dff and t == "lcout":
                    continue      # through the FF
                if f in pins and t in pins:
                    self.fan[pins[f]].append((pins[t], lo, hi))
            if ty == "PRE_IO" or ty == "IO_PAD":
                continue
            if ty == "LogicCell40" and not dff:
                continue
            if ty in ("LogicCell40", "SB_RAM40_4K", "SB_SPRAM256KA") or ty.startswith("SB_MAC16"):
                for p, n in pins.items():
                    if p in self.setup.get(ty, {}) or p == "clk" or (ty != "LogicCell40" and p not in ("RDATA",)):
                        self.sinks[n].append((ty, name, p))

    def root(self, n):
        m = re.search(r"glb_netwk_(\d)", n)
        if m:
            return "GLB" + m[1]    # one net, the stubs at each tile are all it
        while n in self.rep:
            n = self.rep[n]
        return n

    def arrivals(self, src):
        """(lo, hi) at every net reachable from src, combinational only"""
        order, state, stack = [], {}, [(src, iter(self.fan.get(src, ())))]
        state[src] = 1
        while stack:
            n, it = stack[-1]
            for t, _, _ in it:
                if t not in state:
                    state[t] = 1
                    stack.append((t, iter(self.fan.get(t, ()))))
                    break
            else:
                order.append(n)
                stack.pop()
        lo, hi = {src: 0.0}, {src: 0.0}
        for n in reversed(order):
            for t, a, b in self.fan.get(n, ()):
                x, y = lo[n] + a, hi[n] + b
                if t not in lo or x < lo[t]:
                    lo[t] = x
                if t not in hi or y > hi[t]:
                    hi[t] = y
        return lo, hi


def spi_skew(tv, corner):
    """clock pin to pad, sck and mosi: (sck, mosi). mosi - sck is the hold the ESP's filtered SCLK eats"""
    g = Graph(tv, corner)
    rev = defaultdict(list)
    for a, l in g.fan.items():
        for t, _, hi in l:
            rev[t].append((a, hi))
    ffq = {p.get("lcout"): p for p in g.ff.values()}
    glb = {}
    for k in range(8):
        for n, d in g.arrivals("GLB%d" % k)[1].items():
            glb.setdefault(n, d)

    def back(pad):
        best, st, seen = 0.0, [(pad, 0.0)], {}
        while st:
            n, d = st.pop()
            if n in ffq:
                best = max(best, d + glb.get(ffq[n].get("clk"), 0.0))
                continue
            for a, h in rev.get(n, ()):
                if seen.get(a, -1) < d + h:
                    seen[a] = d + h
                    st.append((a, d + h))
        return best
    return back("spi_sck"), back("spi_mosi")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asc")
    ap.add_argument("tv")
    ap.add_argument("--corner", type=int, default=1)
    ap.add_argument("--pads", default="extbus_")
    ap.add_argument("--spi", action="store_true", help="only print the sck and mosi pad delays and mosi - sck")
    a = ap.parse_args()
    if a.spi:
        k, m = spi_skew(a.tv, a.corner)
        print("sck %.0f mosi %.0f mosi-sck %.0f ps" % (k, m, m - k))
        return
    sym = load_syms(a.asc)
    g = Graph(a.tv, a.corner)
    su = g.setup["LogicCell40"]
    glb = {}
    for i in range(8):
        lo, hi = g.arrivals("GLB%d" % i)
        glb[i] = hi
    for pad in sorted(g.pads):
        if not re.search(a.pads, pad):
            continue
        lo, hi = g.arrivals(pad)
        for p in g.pads:
            if p != pad and p in hi:
                print(pad, "pad", p, round(lo[p]), round(hi[p]), "-", 0, 0)
        for n in hi:
            for ty, name, pin in g.sinks.get(n, ()):
                ff = g.ff.get(name)
                q = sym.get(ff.get("lcout"), ff.get("lcout")) if ff else name
                ck = ff.get("clk") if ff else None
                src = next((i for i in glb if ck in glb[i]), -1)
                print(pad, pin, q, round(lo[n]), round(hi[n]), "g%d" % src,
                      round(glb[src][ck]) if src >= 0 else 0, round(su.get(pin, 0)) if ff else 0)
    for i in range(8):
        drv = [p for p in g.pads if "GLB%d" % i in g.arrivals(p)[1]]
        print("#global", i, "from pads", " ".join(sorted(drv)), "sinks",
              sum(1 for n in glb[i] for s_ in g.sinks.get(n, ()) if s_[2] == "clk"))


if __name__ == "__main__":
    main()
