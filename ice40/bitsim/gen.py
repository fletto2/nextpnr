#!/usr/bin/env python3
"""gen.py X.net OUT.c [--part N]: emit straight-line levelized C for a flat netlist.
Combinational logic is cut into partitions in topological order; a partition only
runs when one of its inputs changed. --part 0 runs everything on every delta."""
import argparse, heapq, os, re, sys
from collections import defaultdict
from functools import lru_cache
from asc2net import spram_pins, mac_pins

HERE = os.path.dirname(os.path.abspath(__file__))
NOOUT = -1
GROUP = 16      # partitions under one summary flag
FFCHUNK = 16    # FFs under one flag


class Net:
    """netlist with aliases resolved; constants are nets 0 and 1"""

    def __init__(self, path):
        self.parent = {}
        self.luts = {}          # out -> (ins list, truth table int)
        self.ffs = []
        self.ios, self.rams, self.sprams, self.macs = [], [], [], []
        self.plls = []
        self.refpads = []
        self.nnets = 0
        for line in open(path):
            f = line.split()
            k = f[0]
            if k == "nets":
                self.nnets = int(f[1])
            elif k == "alias":
                self.union(int(f[1]), int(f[2]))
            elif k == "lut":
                ins = [int(a) for a in f[2:6]]
                self.luts[int(f[1])] = (ins, int(f[6], 16))
            elif k == "carry":
                # majority of (i1, i2, cin), a plain 3-input function from here on
                self.luts[int(f[1])] = ([int(f[2]), int(f[3]), int(f[4])], 0xE8)
            elif k == "ff":
                q, d, clk, cen, sr, neg = map(int, f[1:7])
                self.ffs.append(dict(q=q, d=d, clk=clk, cen=cen, sr=sr, neg=neg, setv=int(f[7]),
                                     asyn=int(f[8]), inv=0))
            elif k == "io":
                n = list(map(int, f[3:13]))
                self.ios.append(dict(name=f[1], typ=int(f[2], 2), neg=n[0], pad=n[1], din0=n[2], din1=n[3], d0=n[4],
                                     d1=n[5], oe=n[6], cen=n[7], iclk=n[8], oclk=n[9]))
                # latch dropped with the pin types that need it
                self.ios[-1]["latch"] = int(f[13])
            elif k == "ram":
                x, y, rm, wm, nr, nw = map(int, f[1:7])
                a = list(map(int, f[7:-1]))
                self.rams.append(dict(x=x, y=y, rm=rm, wm=wm, nr=nr, nw=nw, rclk=a[0], rclke=a[1], re=a[2],
                                      wclk=a[3], wclke=a[4], we=a[5], raddr=a[6:17], waddr=a[17:28],
                                      mask=a[28:44], wdata=a[44:60], rdata=a[60:76], init=f[-1]))
            elif k == "spram":
                p = dict(zip(spram_pins(), map(int, f[4:])))
                self.sprams.append(dict(idx=int(f[3]), p=p))
            elif k == "mac":
                h, b = line.split(":")
                hf = h.split()
                cfg = {a: int(c) for a, c in (kv.split("=") for kv in hf[4:])}
                self.macs.append(dict(x=int(hf[1]), y=int(hf[2]), cfg=cfg, p=dict(zip(mac_pins(), map(int, b.split())))))
            elif k == "pll":
                self.plls.append((int(f[1]), int(f[2])))
            elif k == "refpad":
                self.refpads.append(f[1])

    def find(self, a):
        while self.parent.get(a, a) != a:
            self.parent[a] = self.parent.get(self.parent[a], self.parent[a])
            a = self.parent[a]
        return a

    def union(self, a, b):
        a, b = self.find(a), self.find(b)
        if a == b:
            return
        if a in (0, 1):
            self.parent[b] = a
        else:
            self.parent[a] = b

    def new_net(self):
        self.nnets += 1
        return self.nnets - 1


def tt_cofactor(tt, k, i, val):
    """cofactor of a k-input table on input i, leaving a k-1 input table"""
    r, o = 0, 0
    for idx in range(1 << k):
        if (idx >> i & 1) == val:
            r |= (tt >> idx & 1) << o
            o += 1
    return r


def tt_reduce(ins, tt):
    """fold constants, merge duplicate inputs, drop inputs the table ignores"""
    ins = list(ins)
    tt &= (1 << (1 << len(ins))) - 1
    ch = True
    while ch:
        ch = False
        for i, n in enumerate(ins):
            if n in (0, 1):
                tt = tt_cofactor(tt, len(ins), i, n)
                ins.pop(i)
                ch = True
                break
            if n in ins[:i]:
                j = ins.index(n)
                k = len(ins)
                r, o = 0, 0
                # keep the rows where both inputs agree
                for idx in range(1 << k):
                    if idx >> i & 1 == idx >> j & 1:
                        r |= (tt >> idx & 1) << o
                        o += 1
                tt = r
                ins.pop(i)
                ch = True
                break
            lo, hi = tt_cofactor(tt, len(ins), i, 0), tt_cofactor(tt, len(ins), i, 1)
            if lo == hi:
                tt = lo
                ins.pop(i)
                ch = True
                break
    return ins, tt


@lru_cache(maxsize=None)
def synth(tt, k):
    """(cost, expression over a0..a{k-1}) for a table; values are 0/1, bit ops only"""
    if k == 0:
        return 0, str(tt & 1)
    best = None
    for i in range(k):
        lo, hi = tt_cofactor(tt, k, i, 0), tt_cofactor(tt, k, i, 1)
        rename = [j for j in range(k) if j != i]

        def sub(t):
            c, e = synth(t, k - 1)
            return c, re.sub(r"a(\d)", lambda m: "b%d" % rename[int(m[1])], e)
        x = "b%d" % i
        full = (1 << (1 << (k - 1))) - 1
        cl, el = sub(lo)
        ch, eh = sub(hi)
        if lo == hi:
            cand = (cl, el)
        elif lo == 0 and hi == full:
            cand = (0, x)
        elif lo == full and hi == 0:
            cand = (1, "(%s^1)" % x)
        elif lo == 0:
            cand = (ch + 1, "(%s&%s)" % (x, eh))
        elif hi == 0:
            cand = (cl + 2, "((%s^1)&%s)" % (x, el))
        elif hi == full:
            cand = (cl + 1, "(%s|%s)" % (x, el))
        elif lo == full:
            cand = (ch + 2, "((%s^1)|%s)" % (x, eh))
        elif lo ^ hi == full:
            cand = (cl + 1, "(%s^%s)" % (x, el))
        else:
            cand = (cl + ch + 3, "((%s&%s)|((%s^1)&%s))" % (x, eh, x, el))
        if best is None or cand[0] < best[0]:
            best = cand
    return best[0], best[1].replace("b", "a")


def lut_expr(ins, tt, names):
    k = len(ins)
    c, e = synth(tt, k)
    # one shift of the table beats a long mux tree
    shift_cost = 2 * k + 1
    if c > shift_cost:
        idx = "|".join(("a%d" % i if i == 0 else "(a%d<<%d)" % (i, i)) for i in range(k))
        e = "((%du>>(%s))&1)" % (tt, idx)
    return re.sub(r"a(\d)", lambda m: names[int(m[1])], e)


class Gen:
    def __init__(self, nl, part):
        self.n = nl
        self.part = part
        self.loops = 0
        self.count = False
        self.act_file = None
        self.act_shift = 2
        self.branchless = 0

    # -- netlist preparation -------------------------------------------------
    def expand_io(self):
        n = self.n
        self.inputs = []         # (name, net)
        self.outputs = []        # (name, net)
        self.bidirs = []
        for io in n.ios:
            t = io["typ"]
            din, dout, oen = t & 3, t >> 2 & 3, t >> 4 & 3
            if io["din0"] != NOOUT and din != 1:
                sys.exit("%s: PIN_TYPE input mode %d not modelled" % (io["name"], din))
            if io["din1"] != NOOUT:
                sys.exit("%s: D_IN_1 not modelled" % io["name"])
            if oen == 0:
                self.inputs.append((io["name"], io["pad"]))
                if io["din0"] != NOOUT:
                    n.union(io["din0"], io["pad"])
                continue
            if dout == 0:
                sys.exit("%s: DDR output not modelled" % io["name"])
            if dout in (1, 3):
                q = n.new_net()
                n.ffs.append(dict(q=q, d=io["d0"], clk=io["oclk"], cen=io["cen"], sr=0, neg=io["neg"], setv=0,
                                  asyn=0, inv=int(dout == 3)))
                src = q
            else:
                src = io["d0"]
            if oen == 1:
                n.union(io["pad"], src)
                self.outputs.append((io["name"], io["pad"]))
                continue
            if oen == 3:
                q = n.new_net()
                n.ffs.append(dict(q=q, d=io["oe"], clk=io["oclk"], cen=io["cen"], sr=0, neg=io["neg"], setv=0,
                                  asyn=0, inv=0))
                oe = q
            else:
                oe = io["oe"]
            if io["din0"] != NOOUT:
                n.union(io["din0"], io["pad"])
            self.bidirs.append(dict(name=io["name"], pad=io["pad"], oe=oe, dout=src))

    def fold(self):
        """constant/identity folding to a fixpoint, FFs included (they all power up 0)"""
        n = self.n
        find = n.find
        ch = True
        while ch:
            ch = False
            for out in list(n.luts):
                ins, tt = n.luts[out]
                r, t = tt_reduce([find(a) for a in ins], tt)
                o = find(out)
                if o in (0, 1):
                    del n.luts[out]
                    continue
                if not r:
                    del n.luts[out]
                    n.union(out, t & 1)
                    ch = True
                elif len(r) == 1 and t == 2:
                    del n.luts[out]
                    n.union(out, r[0])
                    ch = True
                else:
                    if o != out:
                        del n.luts[out]
                    n.luts[o] = (r, t)
            keep = []
            for f in n.ffs:
                f["clk"], f["cen"], f["sr"], f["d"] = find(f["clk"]), find(f["cen"]), find(f["sr"]), find(f["d"])
                setv = f["setv"]
                # q can only hold 0 if every way in writes 0
                sr_quiet = f["sr"] == 0 or setv == 0
                if sr_quiet and not f["inv"] and (f["d"] == 0 or f["cen"] == 0 or f["clk"] in (0, 1)):
                    n.union(f["q"], 0)
                    ch = True
                    continue
                keep.append(f)
            n.ffs = keep
        for f in n.ffs:
            f["q"] = find(f["q"])
        for r in n.rams:
            for k in ("rclk", "rclke", "re", "wclk", "wclke", "we"):
                r[k] = find(r[k])
            for k in ("raddr", "waddr", "mask", "wdata"):
                r[k] = [find(a) for a in r[k]]
            r["rdata"] = [find(a) if a != NOOUT else a for a in r["rdata"]]
        for s in n.sprams:
            s["p"] = {k: (find(v) if v != NOOUT else v) for k, v in s["p"].items()}
        for m in n.macs:
            m["p"] = {k: (find(v) if v != NOOUT else v) for k, v in m["p"].items()}
        self.inputs = [(a, find(b)) for a, b in self.inputs]
        self.outputs = [(a, find(b)) for a, b in self.outputs]
        for b in self.bidirs:
            for k in ("pad", "oe", "dout"):
                b[k] = find(b[k])
        n.plls = [(find(a), find(b) if b != NOOUT else b) for a, b in n.plls]

    def mac_comb_pins(self, m):
        """pins with a combinational path to O (config decides)"""
        c = m["cfg"]
        two = lambda k: c[k + "_0"] | c[k + "_1"] << 1
        a = set() if c["A_REG"] else {"A"}
        b = set() if c["B_REG"] else {"B"}
        cc = set() if c["C_REG"] else {"C"}
        d = set() if c["D_REG"] else {"D"}
        ab = a | b
        dF = set() if c["TOP_8x8_MULT_REG"] else ab
        dG = set() if c["BOT_8x8_MULT_REG"] else ab
        dJ = set() if c["PIPELINE_16x16_MULT_REG1"] else ab
        dL = dF | dG | dJ
        dH = set() if c["PIPELINE_16x16_MULT_REG2"] else dL
        dZ = [b, dG, dH, set()][two("BOTADDSUB_LOWERINPUT")]
        dY = d if c["BOTADDSUB_UPPERINPUT"] else set()
        dX = [a, dF, dH, dZ][two("TOPADDSUB_LOWERINPUT")]
        dW = cc if c["TOPADDSUB_UPPERINPUT"] else set()
        dLCI = {"CI"} if two("BOTADDSUB_CARRYSELECT") == 3 else set()
        dBot = dZ | dY | dLCI | {"ADDSUBBOT", "OLOADBOT"} | d
        dHCI = (dZ | dY | dLCI | {"ADDSUBBOT"}) if two("TOPADDSUB_CARRYSELECT") >= 2 else set()
        dTop = dX | dW | dHCI | {"ADDSUBTOP", "OLOADTOP"} | cc
        oh = [dTop, set(), dF, dH][two("TOPOUTPUT_SELECT")]
        ol = [dBot, set(), dG, dH][two("BOTOUTPUT_SELECT")]
        pins = set()
        for g in oh | ol:
            if g in "ABCD":
                pins |= {"%s_%d" % (g, i) for i in range(16)}
            else:
                pins.add(g)
        return pins

    def build_nodes(self):
        """every combinational node: ('lut', out) / ('mac', i) / ('bidir', i)"""
        n = self.n
        self.nodes = []
        self.drv = {}           # net -> node index
        for out in n.luts:
            self.drv[out] = len(self.nodes)
            self.nodes.append(("lut", out))
        for i, m in enumerate(n.macs):
            idx = len(self.nodes)
            self.nodes.append(("mac", i))
            for k, v in m["p"].items():
                if (k.startswith("O_") or k == "CO") and v not in (NOOUT, 0, 1):
                    self.drv[v] = idx
        for i, b in enumerate(self.bidirs):
            self.drv[b["pad"]] = len(self.nodes)
            self.nodes.append(("bidir", i))

    def node_inputs(self, nd):
        n = self.n
        if nd[0] == "lut":
            return list(n.luts[nd[1]][0])
        if nd[0] == "bidir":
            b = self.bidirs[nd[1]]
            return [b["oe"], b["dout"]]
        m = n.macs[nd[1]]
        # all data pins read the nets; only true comb ones order the schedule
        return [m["p"][p] for p in self.mac_comb_pins(m) if p in m["p"]]

    def node_outputs(self, nd):
        n = self.n
        if nd[0] == "lut":
            return [nd[1]]
        if nd[0] == "bidir":
            return [self.bidirs[nd[1]]["pad"]]
        m = n.macs[nd[1]]
        return [v for k, v in m["p"].items() if (k.startswith("O_") or k == "CO") and v not in (NOOUT, 0, 1)]

    def isolate_state_reads(self):
        """FFs read only combinational nets while they commit: a control or data pin wired
        straight to state gets a buffer node, so pre-edge values survive the commit"""
        n = self.n
        bufs = {}

        def buf(net):
            if net in (0, 1) or net in self.drv or net in self.input_nets:
                return net
            if net not in bufs:
                b = n.new_net()
                bufs[net] = b
                n.luts[b] = ([net], 2)
                self.drv[b] = len(self.nodes)
                self.nodes.append(("lut", b))
            return bufs[net]
        for f in n.ffs:
            f["d"], f["cen"], f["sr"] = buf(f["d"]), buf(f["cen"]), buf(f["sr"])

    def undriven(self):
        """nets something reads and nothing drives: they read 0, like an unrouted input"""
        n = self.n
        drv = set(self.drv) | {f["q"] for f in n.ffs} | self.input_nets | {0, 1, NOOUT}
        for r in n.rams:
            drv |= set(r["rdata"])
        for s in n.sprams:
            drv |= {v for k, v in s["p"].items() if k.startswith("DATAOUT")}
        reads = set()
        for nd in self.nodes:
            reads |= set(self.all_node_reads(nd))
        for f in n.ffs:
            reads |= {f["d"], f["clk"], f["cen"], f["sr"]}
        for r in n.rams:
            for k in ("rclk", "rclke", "re", "wclk", "wclke", "we"):
                reads.add(r[k])
            for k in ("raddr", "waddr", "mask", "wdata"):
                reads |= set(r[k])
        for s in n.sprams:
            reads |= {v for k, v in s["p"].items() if not k.startswith("DATAOUT")}
        return sorted(reads - drv)

    def sweep(self):
        """drop whatever no pad can ever observe"""
        n = self.n
        live = set(b for _, b in self.outputs)
        for b in self.bidirs:
            live |= {b["pad"], b["oe"], b["dout"]}
        ff_by_q = {f["q"]: f for f in n.ffs}
        rd_by_net = defaultdict(list)
        for r in n.rams:
            for a in r["rdata"]:
                rd_by_net[a].append(r)
        for s in n.sprams:
            for k, v in s["p"].items():
                if k.startswith("DATAOUT"):
                    rd_by_net[v].append(s)
        mac_by_net = {}
        for m in n.macs:
            for k, v in m["p"].items():
                if k.startswith("O_") or k == "CO":
                    mac_by_net[v] = m
        work = list(live)
        seen_cells = set()

        def add(a):
            if a not in live and a not in (0, 1, NOOUT):
                live.add(a)
                work.append(a)
        while work:
            a = work.pop()
            if a in n.luts:
                for b in n.luts[a][0]:
                    add(b)
            if a in ff_by_q:
                f = ff_by_q[a]
                for b in (f["d"], f["clk"], f["cen"], f["sr"]):
                    add(b)
            for c in rd_by_net.get(a, ()):
                if id(c) not in seen_cells:
                    seen_cells.add(id(c))
                    if "p" in c:
                        for k, b in c["p"].items():
                            if not k.startswith("DATAOUT"):
                                add(b)
                    else:
                        for k in ("rclk", "rclke", "re", "wclk", "wclke", "we"):
                            add(c[k])
                        for k in ("raddr", "waddr", "mask", "wdata"):
                            for b in c[k]:
                                add(b)
            m = mac_by_net.get(a)
            if m is not None and id(m) not in seen_cells:
                seen_cells.add(id(m))
                for k, b in m["p"].items():
                    if not (k.startswith("O_") or k == "CO"):
                        add(b)
        dead = len(n.luts)
        n.luts = {k: v for k, v in n.luts.items() if k in live}
        self.swept = (dead - len(n.luts), len(n.ffs))
        n.ffs = [f for f in n.ffs if f["q"] in live]
        self.swept = (self.swept[0], self.swept[1] - len(n.ffs))
        self.live_cells = seen_cells
        n.rams = [r for r in n.rams if id(r) in seen_cells]
        n.sprams = [r for r in n.sprams if id(r) in seen_cells]
        n.macs = [r for r in n.macs if id(r) in seen_cells]

    def schedule(self):
        """Tarjan over fan-ins: SCCs come out inputs first, so the output is the level order,
        with cones kept together. Any SCC above one node is a real combinational loop."""
        nodes = self.nodes
        fanin = [[self.drv[a] for a in self.node_inputs(nd) if a in self.drv] for nd in nodes]
        index, low, onstack = {}, {}, set()
        stack, order, cnt = [], [], [0]
        for root in range(len(nodes)):
            if root in index:
                continue
            work = [(root, 0)]
            while work:
                v, i = work.pop()
                if i == 0:
                    index[v] = low[v] = cnt[0]
                    cnt[0] += 1
                    stack.append(v)
                    onstack.add(v)
                rec = False
                for j in range(i, len(fanin[v])):
                    w = fanin[v][j]
                    if w not in index:
                        work.append((v, j + 1))
                        work.append((w, 0))
                        rec = True
                        break
                    if w in onstack:
                        low[v] = min(low[v], index[w])
                if rec:
                    continue
                if low[v] == index[v]:
                    comp = []
                    while True:
                        w = stack.pop()
                        onstack.discard(w)
                        comp.append(w)
                        if w == v:
                            break
                    order.append(comp)
                if work:
                    u = work[-1][0]
                    low[u] = min(low[u], low[v])
        self.order = order
        self.loops = sum(1 for c in order if len(c) > 1 or c[0] in fanin[c[0]])
        self.loop_nodes = sum(len(c) for c in order if len(c) > 1 or c[0] in fanin[c[0]])
        self.fanin = fanin

    def order_by_activity(self):
        """same dependencies, but nodes that wake together get scheduled together, so a
        quiet partition is quiet for real. Only the schedule changes, never the result."""
        act = {}
        for line in open(self.act_file):
            a, c = line.split()
            act[int(a)] = int(c)
        comp_of = {}
        for ci, comp in enumerate(self.order):
            for nd in comp:
                comp_of[nd] = ci

        def bucket(ci):
            m = max(act.get(a, 0) for nd in self.order[ci] for a in self.node_outputs(self.nodes[nd]))
            return -(m.bit_length() // self.act_shift)
        succ = defaultdict(set)
        deg = [0] * len(self.order)
        for ci, comp in enumerate(self.order):
            ins = {comp_of[w] for nd in comp for w in self.fanin[nd]} - {ci}
            deg[ci] = len(ins)
            for w in ins:
                succ[w].add(ci)
        heap = [(bucket(i), i) for i in range(len(self.order)) if deg[i] == 0]
        heapq.heapify(heap)
        out = []
        while heap:
            _, ci = heapq.heappop(heap)
            out.append(self.order[ci])
            for w in succ[ci]:
                deg[w] -= 1
                if deg[w] == 0:
                    heapq.heappush(heap, (bucket(w), w))
        assert len(out) == len(self.order)
        self.order = out

    def partition(self):
        part = self.part
        self.parts = []          # list of dict(nodes=[...], loop=bool)
        cur = []
        for comp in self.order:
            is_loop = len(comp) > 1 or comp[0] in self.fanin[comp[0]]
            if is_loop:
                if cur:
                    self.parts.append(dict(nodes=cur, loop=False))
                    cur = []
                self.parts.append(dict(nodes=list(reversed(comp)), loop=True))
                continue
            cur.append(comp[0])
            if part and len(cur) >= part:
                self.parts.append(dict(nodes=cur, loop=False))
                cur = []
        if cur:
            self.parts.append(dict(nodes=cur, loop=False))
        self.node_part = {}
        for i, p in enumerate(self.parts):
            for nd in p["nodes"]:
                self.node_part[nd] = i

    def analyse_readers(self):
        """per net: partitions that read it, and whether it must live in v[]"""
        n = self.n
        self.readers = defaultdict(set)
        self.stored = set()
        for i, p in enumerate(self.parts):
            for nd in p["nodes"]:
                for a in self.all_node_reads(self.nodes[nd]):
                    if a in self.drv and self.node_part.get(self.drv[a]) == i and not p["loop"]:
                        self.readers[a]   # local unless something outside reads it
                    else:
                        self.readers[a].add(i)
                        self.stored.add(a)
        for f in n.ffs:
            self.stored |= {f["d"], f["cen"], f["sr"], f["clk"], f["q"]}
        for r in n.rams:
            for k in ("rclk", "rclke", "re", "wclk", "wclke", "we"):
                self.stored.add(r[k])
            for k in ("raddr", "waddr", "mask", "wdata"):
                self.stored |= set(r[k])
        for s in n.sprams:
            self.stored |= {v for k, v in s["p"].items()}
        for m in n.macs:
            self.stored |= {v for k, v in m["p"].items()}
        for _, a in self.outputs + self.inputs:
            self.stored.add(a)
        for b in self.bidirs:
            self.stored |= {b["pad"], b["oe"], b["dout"]}
        for g, l in n.plls:
            self.stored |= {g, l}
        self.stored -= {0, 1, NOOUT}

    def all_node_reads(self, nd):
        if nd[0] == "mac":
            m = self.n.macs[nd[1]]
            return [v for k, v in m["p"].items() if not (k.startswith("O_") or k == "CO")]
        return self.node_inputs(nd)

    def number_nets(self):
        idx = {0: 0, 1: 1}
        order = []
        for p in self.parts:
            for nd in p["nodes"]:
                order += self.node_outputs(self.nodes[nd])
        for a in sorted(self.stored):
            if a not in order:
                order.append(a)
        seen = set()
        for a in order:
            if a in self.stored and a not in seen and a not in (0, 1):
                seen.add(a)
                idx[a] = len(idx)
        self.idx = idx
        self.nv = len(idx)

    # -- emission -------------------------------------------------------------
    def v(self, a):
        if a in (0, 1):
            return str(a)
        if a in self.idx:
            return "v[%d]" % self.idx[a]
        return "n%d" % a

    def dset(self, p):
        return "D[%d]=1;G[%d]=1;" % (p, p // GROUP)

    def mark(self, a, skip=None):
        """statements raising the dirty flags of everything reading net a"""
        if not self.part:
            return ""
        m = "".join(self.dset(p) for p in sorted(self.readers.get(a, ())) if p != skip)
        return m + "".join("F[%d]=1;" % k for k in sorted(self.flagc.get(a, ())))

    def plan_flags(self):
        """FFs and memory ports re-evaluate only when an input moved since their last edge:
        with the same inputs a re-load changes nothing. Accumulators are the exception."""
        n = self.n
        self.nflag = 0
        self.flagc = defaultdict(set)
        pos = {}
        for i, p in enumerate(self.parts):
            for nd in p["nodes"]:
                for a in self.node_outputs(self.nodes[nd]):
                    pos[a] = i

        def alloc(nets):
            k = self.nflag
            self.nflag += 1
            for a in nets:
                if a not in (0, 1, NOOUT):
                    self.flagc[a].add(k)
            return k
        self.chunks = {}
        for key, m in self.dom:
            groups = defaultdict(list)
            for f in m["ff"]:
                groups[(f["cen"], f["sr"], f["asyn"])].append(f)
            out = []
            for (cen, sr, asyn), fs in sorted(groups.items(), key=lambda kv: kv[0]):
                fs.sort(key=lambda f: pos.get(f["d"], -1))
                for i in range(0, len(fs), FFCHUNK):
                    c = fs[i:i + FFCHUNK]
                    nets = [cen, sr] + [f["d"] for f in c]
                    out.append(dict(cen=cen, sr=sr, asyn=asyn, ffs=c, flag=alloc(nets)))
            self.chunks[key] = out
        for r in n.rams:
            r["rflag"] = alloc(r["raddr"] + [r["re"], r["rclke"]])
            r["cflag"] = alloc([])
        for s in n.sprams:
            s["flag"] = alloc([v for k, v in s["p"].items() if not k.startswith("DATAOUT") and k != "CLOCK"])
            s["cflag"] = alloc([])
        for m in n.macs:
            two = lambda k: m["cfg"][k + "_0"] | m["cfg"][k + "_1"] << 1
            m["flag"] = None
            # rQ/rS feed back on themselves: anything that shows them must run every edge
            if two("TOPOUTPUT_SELECT") >= 2 and two("BOTOUTPUT_SELECT") >= 2:
                m["flag"] = alloc([v for k, v in m["p"].items() if not (k.startswith("O_") or k in ("CO", "CLK"))])

    def gather(self, nets):
        """nets as an unsigned, bit i = nets[i]; unconnected reads 0"""
        t = []
        for i, a in enumerate(nets):
            if a == 0:
                continue
            e = self.v(a)
            t.append(e if i == 0 else "((unsigned)%s<<%d)" % (e, i))
        return "|".join(t) if t else "0"

    def emit_node(self, o, nd, pi, loop):
        kind = self.nodes[nd]
        if kind[0] == "lut":
            out = kind[1]
            ins, tt = self.n.luts[out]
            e = lut_expr(ins, tt, [self.v(a) for a in ins])
            if out in self.idx:
                if loop:
                    o.append("\tt=%s; ch|=t^%s; %s=t;" % (e, self.v(out), self.v(out)))
                else:
                    self.emit_store(o, out, e, pi)
            else:
                o.append("\tconst unsigned n%d=%s;" % (out, e))
        elif kind[0] == "bidir":
            i = kind[1]
            b = self.bidirs[i]
            e = "(%s ? %s : (uint8_t)(ext_en[%d] & ext_val[%d]))" % (self.v(b["oe"]), self.v(b["dout"]), i, i)
            if loop:
                o.append("\tt=%s; ch|=t^%s; %s=t;" % (e, self.v(b["pad"]), self.v(b["pad"])))
            else:
                self.emit_store(o, b["pad"], e, pi)
        else:
            i = kind[1]
            m = self.n.macs[i]
            o.append("\t{ mac_res r; mac%d_eval(&r);" % i)
            for j in range(32):
                a = m["p"]["O_%d" % j]
                if a not in (NOOUT, 0, 1):
                    self.emit_store(o, a, "(r.o>>%d)&1" % j, pi)
            a = m["p"]["CO"]
            if a not in (NOOUT, 0, 1):
                self.emit_store(o, a, "r.co", pi)
            o.append("\t}")

    def emit_store(self, o, out, e, pi):
        m = self.mark(out, pi)
        if m and m.count("=1;") <= self.branchless:
            # a net toggling every few runs mispredicts more than these stores cost
            o.append("\t{unsigned t=%s, c=t^%s; %s=t;%s}" % (e, self.v(out), self.v(out), m.replace("=1;", "|=c;")))
        elif m:
            o.append("\t{unsigned t=%s; if(t!=%s){%s=t;%s}}" % (e, self.v(out), self.v(out), m))
        else:
            o.append("\t%s=%s;" % (self.v(out), e))

    def emit_parts(self, o):
        for i, p in enumerate(self.parts):
            o.append("static void p%d(void)\n{" % i)
            if p["loop"]:
                o.append("\tint it; unsigned t, ch;\n\tfor (it = 0; it < LOOP_MAX; it++) {\n\tch = 0;")
                for nd in p["nodes"]:
                    self.emit_node(o, nd, i, True)
                o.append("\tif (!ch) break;\n\t}\n\tif (it == LOOP_MAX) bs_stats.loop_fail++;")
                outs = [a for nd in p["nodes"] for a in self.node_outputs(self.nodes[nd])]
                # the settled value decides, so tell every reader
                o.append("\t" + "".join(self.mark(a, i) for a in outs))
            else:
                for nd in p["nodes"]:
                    self.emit_node(o, nd, i, False)
            o.append("}\n")
        if self.count:
            o.append("static uint64_t C[NP];\nvoid bitsim_counts(const char *path)\n{\n\tFILE *f = fopen(path, \"w\");\n\tif (!f) return;")
            for i, p in enumerate(self.parts):
                for nd in p["nodes"]:
                    for a in self.node_outputs(self.nodes[nd]):
                        o.append('\tfprintf(f, "%d %%llu\\n", (unsigned long long)C[%d]);' % (a, i))
            o.append("\tfclose(f);\n}\n")
        o.append("static void settle(void)\n{")
        if self.part:
            for g in range(0, len(self.parts), GROUP):
                o.append("\tif (G[%d]) {\n\t\tG[%d]=0;" % (g // GROUP, g // GROUP))
                for i in range(g, min(g + GROUP, len(self.parts))):
                    c = "C[%d]++; " % i if self.count else ""
                    o.append("\t\tif (D[%d]) { D[%d]=0; %sp%d(); }" % (i, i, c, i))
                o.append("\t}")
        else:
            for i in range(len(self.parts)):
                o.append("\tp%d();" % i)
        o.append("}\n")

    def clock_domains(self):
        """(net, polarity) -> members; polarity 1 = rising"""
        n = self.n
        dom = defaultdict(lambda: dict(ff=[], rd=[], wr=[], mac=[], spram=[], asr=[], mrst=[]))
        for f in n.ffs:
            if f["clk"] not in (0, 1):
                dom[(f["clk"], 0 if f["neg"] else 1)]["ff"].append(f)
            if f["asyn"] and f["sr"] not in (0, 1):
                dom[(f["sr"], 1)]["asr"].append(f)
            elif f["asyn"] and f["sr"] == 1:
                sys.exit("FF held in async reset forever: fold not done")
        for r in n.rams:
            if r["rclk"] not in (0, 1):
                dom[(r["rclk"], 0 if r["nr"] else 1)]["rd"].append(r)
            if r["wclk"] not in (0, 1):
                dom[(r["wclk"], 0 if r["nw"] else 1)]["wr"].append(r)
        for s in n.sprams:
            if s["p"]["SLEEP"] not in (0, 1) or s["p"]["POWEROFF"] not in (0, 1):
                sys.exit("SPRAM %d: dynamic SLEEP/POWEROFF not modelled" % s["idx"])
            c = s["p"]["CLOCK"]
            if c not in (0, 1):
                dom[(c, 1)]["spram"].append(s)
        for i, m in enumerate(n.macs):
            c = m["p"]["CLK"]
            if c not in (0, 1):
                dom[(c, 0 if m["cfg"]["NEG_TRIGGER"] else 1)]["mac"].append(i)
            for k in ("IRSTTOP", "IRSTBOT", "ORSTTOP", "ORSTBOT"):
                if m["p"][k] != 0:
                    dom[(m["p"][k], 1)]["mrst"].append((i, k))
        self.dom = sorted(dom.items())
        self.clk_nets = sorted(set(k[0] for k, _ in self.dom))
        for a in self.clk_nets:
            self.stored.add(a)

    def ff_load(self, f, q_expr):
        q = self.v(f["q"])
        m = self.mark(f["q"])
        if m and m.count("=1;") <= self.branchless:
            return "{unsigned t=%s, c=t^%s; %s=t;%s}" % (q_expr, q, q, m.replace("=1;", "|=c;"))
        if m:
            return "{unsigned t=%s; if(t!=%s){%s=t;%s}}" % (q_expr, q, q, m)
        return "%s=%s;" % (q, q_expr)

    def emit_ff_chunk(self, o, c):
        cen, sr = c["cen"], c["sr"]
        dexpr = lambda f: self.v(f["d"]) if not f["inv"] else "(%s^1)" % self.v(f["d"])
        srv = self.v(sr)
        if self.part:
            o.append("\tif (F[%d]) { F[%d]=0;" % (c["flag"], c["flag"]))
        if sr != 0 and c["asyn"] and cen != 1:
            # async sr ignores cen
            o.append("\tif (%s) {" % srv)
            o += ["\t\t" + self.ff_load(f, str(f["setv"])) for f in c["ffs"]]
            o.append("\t} else if (%s) {" % self.v(cen))
            o += ["\t\t" + self.ff_load(f, dexpr(f)) for f in c["ffs"]]
            o.append("\t}")
        else:
            if cen != 1:
                o.append("\tif (%s) {" % self.v(cen))
            for f in c["ffs"]:
                e = "%s?%d:%s" % (srv, f["setv"], dexpr(f)) if sr != 0 else dexpr(f)
                o.append("\t\t" + self.ff_load(f, e))
            if cen != 1:
                o.append("\t}")
        if self.part:
            o.append("\t}")

    def emit_ram(self, o):
        for i, r in enumerate(self.n.rams):
            init = r["init"]
            words = []
            for blk in range(16):
                h = int(init[64 * blk:64 * blk + 64], 16)
                for j in range(16):
                    words.append(h >> (16 * j) & 0xFFFF)
            r["id"] = i
            r["words"] = words
        if self.n.rams:
            o.append("static const uint16_t ram_init[%d][256] = {" % len(self.n.rams))
            for r in self.n.rams:
                o.append("\t{" + ",".join("0x%x" % w for w in r["words"]) + "},")
            o.append("};")

    def ram_read(self, r):
        i = r["id"]
        cond = " && ".join(self.v(a) for a in (r["re"], r["rclke"]) if a != 1) or "1"
        if r["re"] == 0 or r["rclke"] == 0:
            return ""
        s = "\tif (%s) ram_read(%d, %d, %s, %d);\n" % (cond, i, r["rm"], self.gather(r["raddr"]), r["cflag"])
        if self.part:
            s = "\tif (F[%d]) { F[%d]=0;\n%s\t}\n" % (r["rflag"], r["rflag"], s)
        return s

    def ram_write(self, r):
        i = r["id"]
        if r["we"] == 0 or r["wclke"] == 0:
            return ""
        cond = " && ".join(self.v(a) for a in (r["we"], r["wclke"]) if a != 1) or "1"
        return "\tif (%s) { ram_write(%d, %d, %s, %s, %s); F[%d]=1; }\n" % (
            cond, i, r["wm"], self.gather(r["waddr"]), self.gather(r["mask"]), self.gather(r["wdata"]), r["rflag"])

    def ram_commit(self, r):
        """RDATA nets change only here, after the FFs took their pre-edge values"""
        i = r["id"]
        s = ["\tif (F[%d]) { F[%d]=0; unsigned o = ram_out(%d, %d);" % (r["cflag"], r["cflag"], r["rm"], i)]
        for j, a in enumerate(r["rdata"]):
            if a in (NOOUT, 0, 1):
                continue
            m = self.mark(a)
            q = self.v(a)
            if m:
                s.append("\t\t{unsigned t=(o>>%d)&1; if(t!=%s){%s=t;%s}}" % (j, q, q, m))
            else:
                s.append("\t\t%s=(o>>%d)&1;" % (q, j))
        s.append("\t}")
        return "\n".join(s) + "\n"

    def spram_stage(self, s, i):
        p = s["p"]
        g = lambda name, cnt: self.gather([p["%s_%d" % (name, k)] for k in range(cnt)])
        rd = "\tspram_clk(%d, %s, %s, %s, %s, %s, %s, %s, %s, %d);\n" % (
            i, self.v(p["CHIPSELECT"]), self.v(p["WREN"]), self.v(p["STANDBY"]), self.v(p["SLEEP"]),
            self.v(p["POWEROFF"]), g("ADDRESS", 14), g("DATAIN", 16), g("MASKWREN", 4), s["cflag"])
        if self.part:
            rd = "\tif (F[%d]) { F[%d]=0;\n%s\t}\n" % (s["flag"], s["flag"], rd)
        c = ["\tif (F[%d]) { F[%d]=0; unsigned o = spram_out[%d];" % (s["cflag"], s["cflag"], i)]
        for j in range(16):
            a = p["DATAOUT_%d" % j]
            if a in (NOOUT, 0, 1):
                continue
            m = self.mark(a)
            q = self.v(a)
            if m:
                c.append("\t\t{unsigned t=(o>>%d)&1; if(t!=%s){%s=t;%s}}" % (j, q, q, m))
            else:
                c.append("\t\t%s=(o>>%d)&1;" % (q, j))
        c.append("\t}")
        return rd, "\n".join(c) + "\n"

    def emit_macs(self, o):
        for i, m in enumerate(self.n.macs):
            c = m["cfg"]
            p = m["p"]
            two = lambda k: c[k + "_0"] | c[k + "_1"] << 1
            cfgv = ", ".join(str(x) for x in (
                c["A_REG"], c["B_REG"], c["C_REG"], c["D_REG"], c["TOP_8x8_MULT_REG"], c["BOT_8x8_MULT_REG"],
                c["PIPELINE_16x16_MULT_REG1"], c["PIPELINE_16x16_MULT_REG2"], two("TOPOUTPUT_SELECT"),
                two("TOPADDSUB_LOWERINPUT"), c["TOPADDSUB_UPPERINPUT"], two("TOPADDSUB_CARRYSELECT"),
                two("BOTOUTPUT_SELECT"), two("BOTADDSUB_LOWERINPUT"), c["BOTADDSUB_UPPERINPUT"],
                two("BOTADDSUB_CARRYSELECT"), c["MODE_8x8"], c["A_SIGNED"], c["B_SIGNED"]))
            g = lambda name: self.gather([p["%s_%d" % (name, k)] for k in range(16)])
            o.append("static inline void mac%d_eval(mac_res *r)\n{" % i)
            o.append("\tmac_comb(%s, &mac[%d], r, %s, %s, %s, %s, %s, %s, %s, %s, %s);" % (
                cfgv, i, g("A"), g("B"), g("C"), g("D"), self.v(p["ADDSUBTOP"]), self.v(p["ADDSUBBOT"]),
                self.v(p["OLOADTOP"]), self.v(p["OLOADBOT"]), self.v(p["CI"])))
            o.append("}")
            o.append("static inline void mac%d_clk(void)\n{" % i)
            if self.part and m["flag"] is not None:
                # three register stages to flush after the inputs settle
                o.append("\tstatic uint8_t busy;\n\tif (F[%d]) { F[%d]=0; busy=4; }\n\tif (!busy) return;\n\tbusy--;" % (m["flag"], m["flag"]))
            o.append("\tmac_res r; mac%d_eval(&r);" % i)
            o.append("\tmac_step(%d, &mac[%d], &r, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s);" % (
                c["MODE_8x8"], i, g("A"), g("B"), g("C"), g("D"), self.v(p["CE"]), self.v(p["AHOLD"]), self.v(p["BHOLD"]),
                self.v(p["CHOLD"]), self.v(p["DHOLD"]), self.v(p["OHOLDTOP"]), self.v(p["OHOLDBOT"]),
                self.v(p["IRSTTOP"]), self.v(p["IRSTBOT"]), self.v(p["ORSTTOP"]), self.v(p["ORSTBOT"])))
            if self.part:
                pp = self.mac_part.get(i)
                if pp is not None:
                    o.append("\t" + self.dset(pp))
            o.append("}\n")

    def emit(self, path):
        n = self.n
        o = []
        o.append('#include "bitsim.h"\n#include <stdio.h>\n#include <string.h>\n')
        o.append("#define NV %d\n#define NP %d\n#define NSPRAM %d\n#define NMAC %d\n#define NEXT %d\n#define NRAM %d\n" % (
            self.nv, len(self.parts), len(n.sprams), max(1, len(n.macs)), max(1, len(self.bidirs)), len(n.rams)))
        o.append("static uint8_t v[NV];\nstatic uint8_t D[NP], G[NP / %d + 1], F[%d];\n" % (GROUP, max(1, self.nflag)))
        o.append(open(os.path.join(HERE, "runtime.inc")).read())
        self.mac_part = {}
        for i, nd in enumerate(self.nodes):
            if nd[0] == "mac" and i in self.node_part:
                self.mac_part[nd[1]] = self.node_part[i]
        self.emit_ram(o)
        self.emit_macs(o)
        self.emit_parts(o)
        # one function per stage per domain
        for di, (key, m) in enumerate(self.dom):
            o.append("static void d%d_rd(void)\n{" % di)
            for r in m["rd"]:
                o.append(self.ram_read(r).rstrip("\n"))
            stages = [self.spram_stage(s, n.sprams.index(s)) for s in m["spram"]]
            for st in stages:
                o.append(st[0].rstrip("\n"))
            for i in m["mac"]:
                o.append("\tmac%d_clk();" % i)
            o.append("}\nstatic void d%d_wr(void)\n{" % di)
            for r in m["wr"]:
                o.append(self.ram_write(r).rstrip("\n"))
            o.append("}\nstatic void d%d_ff(void)\n{" % di)
            for c in self.chunks[key]:
                self.emit_ff_chunk(o, c)
            for f in m["asr"]:
                q = self.v(f["q"])
                o.append("\tif(%s!=%d){%s=%d;%s}" % (q, f["setv"], q, f["setv"], self.mark(f["q"])))
            for i, k in m["mrst"]:
                o.append("\tmac_rst_%s(&mac[%d]);" % (k.lower(), i))
                if self.part and i in self.mac_part:
                    o.append("\t" + self.dset(self.mac_part[i]))
            o.append("}\nstatic void d%d_commit(void)\n{" % di)
            for r in m["rd"]:
                o.append(self.ram_commit(r).rstrip("\n"))
            for st in stages:
                o.append(st[1].rstrip("\n"))
            o.append("}\n")
        o.append("static uint8_t pv[%d];\n" % max(1, len(self.clk_nets)))
        o.append("int bitsim_eval(void)\n{\n\tint it;\n\tbs_stats.evals++;\n\tfor (it = 0; it < DELTA_MAX; it++) {")
        o.append("\t\tunsigned long f = 0;\n\t\tsettle();")
        for ci, c in enumerate(self.clk_nets):
            hi = next((di for di, (k, _) in enumerate(self.dom) if k == (c, 1)), None)
            lo = next((di for di, (k, _) in enumerate(self.dom) if k == (c, 0)), None)
            s = "\t\tif (%s != pv[%d]) { pv[%d] = %s;" % (self.v(c), ci, ci, self.v(c))
            if hi is not None and lo is not None:
                s += " f |= %s ? 1ul<<%d : 1ul<<%d;" % (self.v(c), hi, lo)
            elif hi is not None:
                s += " if (%s) f |= 1ul<<%d;" % (self.v(c), hi)
            else:
                s += " if (!%s) f |= 1ul<<%d;" % (self.v(c), lo)
            s += " }"
            o.append(s)
        o.append("\t\tif (!f) return it;\n\t\tbs_stats.deltas++;")
        for st in ("rd", "wr", "ff", "commit"):
            for di in range(len(self.dom)):
                o.append("\t\tif (f & 1ul<<%d) d%d_%s();" % (di, di, st))
        o.append("\t}\n\tbs_stats.delta_fail++;\n\treturn -1;\n}\n")
        # pins
        names = [a for a, _ in self.inputs] + [b["name"] for b in self.bidirs] + [a for a, _ in self.outputs] + \
            ["clk50", "pll_lock"]
        o.append("const char *const bitsim_pin_names[BS_NPINS] = {" + ",".join('"%s"' % a for a in names) + "};\n")
        o.append("void bitsim_set(int pin, int val)\n{\n\tval = !!val;\n\tswitch (pin) {")
        ids = {a: i for i, a in enumerate(names)}
        for a, net in self.inputs:
            if net in self.idx:
                o.append("\tcase %d: if (%s != val) { %s = val; %s } break;" % (
                    ids[a], self.v(net), self.v(net), self.mark(net)))
        for i, b in enumerate(self.bidirs):
            part = self.node_part[self.drv[b["pad"]]]
            o.append("\tcase %d: ext_en[%d] = 1; ext_val[%d] = val; %s break;" % (ids[b["name"]], i, i, self.dset(part)))
        g, l = n.plls[0]
        o.append("\tcase %d: if (%s != val) { %s = val; %s } break;" % (
            ids["clk50"], self.v(g), self.v(g), self.mark(g)))
        if l in self.idx:
            o.append("\tcase %d: if (%s != val) { %s = val; %s } break;" % (
                ids["pll_lock"], self.v(l), self.v(l), self.mark(l)))
        o.append("\tdefault: break;\n\t}\n}\n")
        o.append("void bitsim_release(int pin)\n{\n\tswitch (pin) {")
        for i, b in enumerate(self.bidirs):
            part = self.node_part[self.drv[b["pad"]]]
            o.append("\tcase %d: ext_en[%d] = 0; %s break;" % (ids[b["name"]], i, self.dset(part)))
        o.append("\tdefault: break;\n\t}\n}\n")
        o.append("int bitsim_get(int pin)\n{\n\tswitch (pin) {")
        for a, net in self.inputs + self.outputs:
            o.append("\tcase %d: return %s;" % (ids[a], self.v(net)))
        for b in self.bidirs:
            o.append("\tcase %d: return %s;" % (ids[b["name"]], self.v(b["pad"])))
        o.append("\tdefault: return 0;\n\t}\n}\n")
        o.append("int bitsim_driven(int pin)\n{\n\tswitch (pin) {")
        for b in self.bidirs:
            o.append("\tcase %d: return %s;" % (ids[b["name"]], self.v(b["oe"])))
        for a, net in self.outputs:
            o.append("\tcase %d: return 1;" % ids[a])
        o.append("\tdefault: return 0;\n\t}\n}\n")
        o.append("int bitsim_contention(void)\n{\n\tint c = 0;")
        for i, b in enumerate(self.bidirs):
            o.append("\tc += %s && ext_en[%d];" % (self.v(b["oe"]), i))
        o.append("\treturn c;\n}\n")
        o.append("void bitsim_reset(void)\n{\n\tmemset(v, 0, sizeof v);\n\tv[1] = 1;\n\tmemset(pv, 0, sizeof pv);")
        o.append("\tmemset(D, 1, sizeof D);\n\tmemset(G, 1, sizeof G);\n\tmemset(F, 1, sizeof F);\n\tmemset(ext_en, 0, sizeof ext_en);\n\tmemset(ext_val, 0, sizeof ext_val);")
        o.append("\tmemset(spram, 0, sizeof spram);\n\tmemset(spram_out, 0, sizeof spram_out);\n\tmemset(mac, 0, sizeof mac);")
        if n.rams:
            o.append("\tmemcpy(ram, ram_init, sizeof ram);\n\tmemset(ram_rd, 0, sizeof ram_rd);")
        o.append("\tmemset(&bs_stats, 0, sizeof bs_stats);\n}\n")
        o.append("void bitsim_spram_load(int cell, const uint16_t *words, int n)\n{\n\t(void)words; (void)n;\n\tswitch (cell) {")
        for i, s in enumerate(n.sprams):
            o.append("\tcase %d: memcpy(spram[%d], words, (size_t)n * 2); break;" % (s["idx"], i))
        o.append("\tdefault: break;\n\t}\n}\n")
        with open(path, "w") as f:
            f.write("\n".join(o))
        h = ["/* generated by gen.py */", "#ifndef BITSIM_PINS_H", "#define BITSIM_PINS_H"]
        for a in names:
            h.append("#define BS_%s %d" % (re.sub(r"\W", "_", a).rstrip("_"), ids[a]))
        h.append("#define BS_NPINS %d" % len(names))
        h.append("#endif")
        with open(os.path.join(os.path.dirname(path) or ".", "bitsim_pins.h"), "w") as f:
            f.write("\n".join(h) + "\n")

    def run(self, path):
        n = self.n
        self.expand_io()
        self.fold()
        self.input_nets = {b for _, b in self.inputs}
        for g, l in n.plls:
            self.input_nets |= {g, l}
        self.build_nodes()
        self.sweep()
        self.build_nodes()
        self.isolate_state_reads()
        self.schedule()
        if self.act_file:
            self.order_by_activity()
        self.partition()
        self.analyse_readers()
        self.clock_domains()
        self.plan_flags()
        self.number_nets()
        self.emit(path)
        print("%d comb nodes, %d FF, %d RAM, %d SPRAM, %d MAC, %d stored nets, %d partitions, %d clock nets, %d domains" % (
            len(self.nodes), len(n.ffs), len(n.rams), len(n.sprams), len(n.macs), self.nv, len(self.parts),
            len(self.clk_nets), len(self.dom)))
        print("swept %d LUTs, %d FFs no pad can see; combinational loops: %d (%d nodes); undriven read nets: %d" % (
            self.swept[0], self.swept[1], self.loops, self.loop_nodes, len(self.undriven())))
        for k, m in self.dom:
            print("  domain net %d %s: %d FF, %d async, %d RAM rd, %d RAM wr, %d SPRAM, %d MAC" % (
                k[0], "pos" if k[1] else "neg", len(m["ff"]), len(m["asr"]), len(m["rd"]), len(m["wr"]),
                len(m["spram"]), len(m["mac"])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("net")
    ap.add_argument("out")
    ap.add_argument("--part", type=int, default=16)
    ap.add_argument("--ffchunk", type=int, default=FFCHUNK)
    ap.add_argument("--count", action="store_true", help="emit bitsim_counts(); use with --part 1")
    ap.add_argument("--act", help="counts from a --count run, orders the schedule by activity")
    ap.add_argument("--act-shift", type=int, default=2)
    ap.add_argument("--branchless", type=int, default=6, help="branch-free dirty marks up to this many flags per net")
    a = ap.parse_args()
    globals()["FFCHUNK"] = a.ffchunk
    g = Gen(Net(a.net), a.part)
    g.count, g.act_file, g.act_shift, g.branchless = a.count, a.act, a.act_shift, a.branchless
    g.run(a.out)


if __name__ == "__main__":
    main()
