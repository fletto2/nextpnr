#!/usr/bin/env python3
"""checknet.py X.net routed.json: prove the decoded netlist equals the routed design.
Cell by cell at each BEL: every config value compared, and every json net must map
to exactly one decoded net both ways."""
import json, re, sys
from collections import Counter
from asc2net import spram_pins, mac_pins

netf, jsonf = sys.argv[1:3]
top = json.load(open(jsonf))["modules"]["top"]
cells = top["cells"]

parent = {}
def find(a):
    while parent.get(a, a) != a:
        parent[a] = parent.get(parent[a], parent[a]); a = parent[a]
    return a
def union(a, b):
    a, b = find(a), find(b)
    if a == b: return
    if a in (0, 1): parent[b] = a
    else: parent[a] = b

dec = {}
lc = {}
carry = {}
for line in open(netf):
    f = line.split()
    if f[0] == "alias": union(int(f[1]), int(f[2]))
    elif f[0] == "lut":
        o, i0, i1, i2, i3 = map(int, f[1:6]); x, y, n = map(int, f[7:10])
        lc[(x, y, n)] = dict(lo=o, i=[i0, i1, i2, i3], init=int(f[6], 16), ff=None, carry=None)
    elif f[0] == "carry":
        carry[tuple(map(int, f[5:8]))] = tuple(map(int, f[1:5]))
    elif f[0] == "ff":
        q, d, clk, cen, sr, neg = map(int, f[1:7]); x, y, n = map(int, f[9:12])
        lc[(x, y, n)]["ff"] = dict(q=q, clk=clk, cen=cen, sr=sr, neg=neg, setb=int(f[7]), asyn=int(f[8]))
    elif f[0] == "ram":
        dec[("ram", int(f[1]), int(f[2]))] = f[3:]
    elif f[0] == "spram":
        dec[("spram", int(f[3]))] = list(map(int, f[4:]))
    elif f[0] == "mac":
        h, b = line.split(":")
        hf = h.split()
        dec[("mac", int(hf[1]), int(hf[2]))] = (dict(kv.split("=") for kv in hf[4:]), list(map(int, b.split())))
    elif f[0] == "io":
        dec[("io", f[1])] = f[2:]
    elif f[0] == "pll":
        dec["pll"] = f[1:]

j2d, d2j = {}, {}
errs = Counter()
def err(msg):
    errs[msg.split(":")[0]] += 1
    if sum(errs.values()) <= 40: print("ERR", msg)

gb = {}
for c in cells.values():
    if c["type"] == "SB_GB":
        i = c["connections"]["USER_SIGNAL_TO_GLOBAL_BUFFER"]
        o = c["connections"]["GLOBAL_BUFFER_OUTPUT"][0]
        if i: gb[o] = i[0]
        else: pll_glb = o   # PLL global, its input is dropped from the json

for c in cells.values():
    if c["type"] == "ICESTORM_LC" and not int(c["parameters"].get("DFF_ENABLE", "0"), 2):
        lo, o = c["connections"]["LO"], c["connections"]["O"]
        if lo and o: gb[lo[0]] = o[0]   # one signal on the die without the FF

def bits(c, p):
    b = c["connections"].get(p, [])
    if not b: return None
    return gb.get(b[0], b[0])

def link(what, jb, dn, dflt):
    """jb json bit (int, '0', '1', 'x' or None), dn decoded net, dflt its unrouted value"""
    if dn == -1:
        return
    dn = find(dn)
    if jb is None or jb == "x":
        if dn != dflt: err("unconnected in json, routed in asc: %s -> %s" % (what, dn))
        return
    if jb in ("0", "1"):
        if dn != int(jb): err("const in json: %s %s dec %s" % (what, jb, dn))
        return
    if jb in j2d and j2d[jb] != dn: err("json net split: %s j%s dec %s/%s" % (what, jb, j2d[jb], dn))
    if dn in d2j and d2j[dn] != jb and dn > 1: err("dec net merged: %s dec %s j%s/%s" % (what, dn, d2j[dn], jb))
    j2d[jb] = dn; d2j[dn] = jb

def pval(c, k, d=0):
    v = c["parameters"].get(k, d)
    if isinstance(v, str):
        return int(re.sub("[xz]", "0", v), 2) if set(v) <= set("01xz") else v
    return v

def canon(pins, init):
    """LUT function over its distinct nets: order and duplicates stop mattering"""
    nets = sorted(set(n for n in pins if n > 1))
    tt = []
    for v in range(1 << len(nets)):
        val = {n: v >> i & 1 for i, n in enumerate(nets)}
        idx = sum((val[n] if n > 1 else n) << k for k, n in enumerate(pins))
        tt.append(init >> idx & 1)
    while nets:   # drop nets the function ignores
        for i, n in enumerate(nets):
            if all(tt[v] == tt[v ^ 1 << i] for v in range(len(tt))):
                tt = [tt[v] for v in range(len(tt)) if not v >> i & 1]
                nets.pop(i)
                break
        else:
            break
    return tuple(nets), tuple(tt)

seen = Counter()
sinks = []
for name, c in cells.items():
    t = c["type"]; bel = c["attributes"].get("NEXTPNR_BEL", "")
    m = re.match(r"X(\d+)/Y(\d+)/(\w+?)_?(\d*)$", bel)
    x, y, kind, z = int(m[1]), int(m[2]), m[3], m[4]
    seen[t] += 1
    if t == "ICESTORM_LC":
        k = (x, y, int(z))
        if k not in lc: err("LC missing: %s" % name); continue
        d = lc[k]
        sinks.append((name, c, d, k))
        if bits(c, "LO") is not None:
            link(name + " LO", bits(c, "LO"), d["lo"], None)
        ff = d["ff"]
        if bool(pval(c, "DFF_ENABLE")) != (ff is not None): err("DFF_ENABLE: %s" % name)
        elif ff:
            link(name + " O", bits(c, "O"), ff["q"], None)
            link(name + " CLK", bits(c, "CLK"), ff["clk"], 0)
            link(name + " CEN", bits(c, "CEN"), ff["cen"], 1)
            link(name + " SR", bits(c, "SR"), ff["sr"], 0)
            for a, b in (("NEG_CLK", "neg"), ("SET_NORESET", "setb"), ("ASYNC_SR", "asyn")):
                if pval(c, a) != ff[b]: err("%s: %s" % (a, name))
        else:
            if bits(c, "O") is not None:
                link(name + " O", bits(c, "O"), d["lo"], None)
        co = bits(c, "COUT")
        if bool(pval(c, "CARRY_ENABLE")) != (k in carry): err("CARRY_ENABLE: %s" % name)
        elif k in carry and co is not None:
            link(name + " COUT", co, carry[k][0], None)
    elif t == "ICESTORM_RAM":
        f = dec.get(("ram", x, y))
        if f is None: err("RAM missing: %s" % name); continue
        rm, wm, nr, nw = map(int, f[:4])
        for a, b in (("READ_MODE", rm), ("WRITE_MODE", wm), ("NEG_CLK_R", nr), ("NEG_CLK_W", nw)):
            if pval(c, a) != b: err("%s: %s" % (a, name))
        nets = list(map(int, f[4:-1]))
        pins = ["RCLK", "RCLKE", "RE", "WCLK", "WCLKE", "WE"] + ["RADDR_%d" % i for i in range(11)] + \
            ["WADDR_%d" % i for i in range(11)] + ["MASK_%d" % i for i in range(16)] + \
            ["WDATA_%d" % i for i in range(16)] + ["RDATA_%d" % i for i in range(16)]
        for p, n in zip(pins, nets):
            link(name + " " + p, bits(c, p), n, 1 if p in ("RCLKE", "WCLKE") else 0)
        data = f[-1]
        for i in range(16):
            v = c["parameters"].get("INIT_%X" % i, "0" * 256).replace("x", "0")
            if int(v, 2) != int(data[64 * i:64 * i + 64], 16): err("RAM INIT: %s %d" % (name, i))
    elif t == "ICESTORM_SPRAM":
        f = dec.get(("spram", int(z)))
        if f is None: err("SPRAM missing: %s" % name); continue
        for p, n in zip(spram_pins(), f):
            link(name + " " + p, bits(c, p), n, 0)
    elif t == "ICESTORM_DSP":
        f = dec.get(("mac", x, y))
        if f is None: err("MAC missing: %s" % name); continue
        cfg, nets = f
        for p, n in zip(mac_pins(), nets):
            link(name + " " + p, bits(c, p), n, 1 if p == "CE" else 0)
        def two(k): return int(cfg[k + "_0"]) | int(cfg[k + "_1"]) << 1
        for k in ("TOPOUTPUT_SELECT", "BOTOUTPUT_SELECT", "TOPADDSUB_LOWERINPUT", "BOTADDSUB_LOWERINPUT",
                  "TOPADDSUB_CARRYSELECT", "BOTADDSUB_CARRYSELECT"):
            if pval(c, k) != two(k): err("%s: %s" % (k, name))
        for k in ("A_REG", "B_REG", "C_REG", "D_REG", "A_SIGNED", "B_SIGNED", "MODE_8x8", "TOP_8x8_MULT_REG",
                  "BOT_8x8_MULT_REG", "PIPELINE_16x16_MULT_REG1", "PIPELINE_16x16_MULT_REG2",
                  "TOPADDSUB_UPPERINPUT", "BOTADDSUB_UPPERINPUT", "NEG_TRIGGER"):
            if pval(c, k) != int(cfg[k]): err("%s: %s" % (k, name))
    elif t == "SB_IO":
        for nm, f in dec.items():
            if isinstance(nm, tuple) and nm[0] == "io" and (int(f[-3]), int(f[-2]), int(f[-1])) == (x, y, int(z)):
                ptype, neg = f[0], int(f[1])
                if pval(c, "PIN_TYPE") != int(ptype, 2): err("PIN_TYPE: %s %s" % (name, ptype))
                if pval(c, "NEG_TRIGGER") != neg: err("IO NEG: %s" % name)
                n = list(map(int, f[2:12]))
                for p, v, dfl in (("D_IN_0", n[1], -1), ("D_IN_1", n[2], -1), ("D_OUT_0", n[3], 0), ("D_OUT_1", n[4], 0),
                                  ("OUTPUT_ENABLE", n[5], 1), ("CLOCK_ENABLE", n[6], 1), ("INPUT_CLK", n[7], 0),
                                  ("OUTPUT_CLK", n[8], 0), ("LATCH_INPUT_VALUE", n[9], 0)):
                    jb = bits(c, p)
                    # clocks and enable are shared with the other IO of the tile
                    if jb is None and p in ("OUTPUT_ENABLE", "OUTPUT_CLK", "INPUT_CLK", "CLOCK_ENABLE"): continue
                    link(name + " " + p, jb, v, dfl)
                break
        else:
            seen["SB_IO unmatched"] += 1
    elif t == "ICESTORM_PLL":
        link(name + " OUT", pll_glb, int(dec["pll"][0]), None)
        link(name + " LOCK", bits(c, "LOCK"), int(dec["pll"][1]), None)

# LUT inputs last: nextpnr permutes them at routing, so compare functions over nets
def want(jb):
    if jb is None or jb == "x": return 0
    if jb in ("0", "1"): return int(jb)
    if jb not in j2d: err("net with no driver seen: %s" % jb); return 0
    return j2d[jb]

for name, c, d, k in sinks:
    a = canon([want(bits(c, "I%d" % i)) for i in range(4)], pval(c, "LUT_INIT"))
    b = canon([find(n) for n in d["i"]], d["init"])
    if a != b: err("LUT function: %s %s %s" % (name, a, b))
    if k in carry:
        co, i1, i2, ci = carry[k]
        # symmetric in its operands, and the router swaps them
        if sorted((want(bits(c, "I1")), want(bits(c, "I2")))) != sorted((find(i1), find(i2))): err("carry operand: %s" % name)
        cin = pval(c, "CIN_SET") if pval(c, "CIN_CONST") else want(bits(c, "CIN"))
        if cin != find(ci): err("carry in: %s %s %s" % (name, cin, find(ci)))

nlc = sum(1 for v in lc.values() if v["ff"])
print("cells", dict(seen))
print("decoded: %d LC, %d FF; json nets mapped %d" % (len(lc), nlc, len(j2d)))
jd = {"LC": sum(1 for c in cells.values() if c["type"] == "ICESTORM_LC"),
      "FF": sum(1 for c in cells.values() if c["type"] == "ICESTORM_LC" and pval(c, "DFF_ENABLE")),
      "CARRY": sum(1 for c in cells.values() if c["type"] == "ICESTORM_LC" and pval(c, "CARRY_ENABLE"))}
print("json:", jd, "decoded carries", len(carry))
print("errors:", dict(errs) or "none")
sys.exit(1 if errs else 0)
