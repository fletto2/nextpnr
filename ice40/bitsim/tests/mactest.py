#!/usr/bin/env python3
"""mactest.py X.net [--random N] [--vectors N] [--seed N]: runtime.inc's MAC16 against yosys' SB_MAC16.

Same random vectors, async reset pulses included, through cells_sim.v under iverilog and the C model,
for every configuration in X.net plus N random ones. Output O and CO compared at every step."""
import argparse, os, random, re, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CELLS = "/opt/oss-fpga/share/yosys/ice40/cells_sim.v"
ONE = ["A_REG", "B_REG", "C_REG", "D_REG", "TOP_8x8_MULT_REG", "BOT_8x8_MULT_REG", "PIPELINE_16x16_MULT_REG1",
       "PIPELINE_16x16_MULT_REG2", "TOPADDSUB_UPPERINPUT", "BOTADDSUB_UPPERINPUT", "MODE_8x8", "A_SIGNED", "B_SIGNED"]
TWO = ["TOPOUTPUT_SELECT", "TOPADDSUB_LOWERINPUT", "TOPADDSUB_CARRYSELECT", "BOTOUTPUT_SELECT",
       "BOTADDSUB_LOWERINPUT", "BOTADDSUB_CARRYSELECT"]
# mac_comb's argument order
ORDER = ["A_REG", "B_REG", "C_REG", "D_REG", "TOP_8x8_MULT_REG", "BOT_8x8_MULT_REG", "PIPELINE_16x16_MULT_REG1",
         "PIPELINE_16x16_MULT_REG2", "TOPOUTPUT_SELECT", "TOPADDSUB_LOWERINPUT", "TOPADDSUB_UPPERINPUT",
         "TOPADDSUB_CARRYSELECT", "BOTOUTPUT_SELECT", "BOTADDSUB_LOWERINPUT", "BOTADDSUB_UPPERINPUT",
         "BOTADDSUB_CARRYSELECT", "MODE_8x8", "A_SIGNED", "B_SIGNED"]
FLAGS = ["CE", "AHOLD", "BHOLD", "CHOLD", "DHOLD", "OHOLDTOP", "OHOLDBOT", "ADDSUBTOP", "ADDSUBBOT", "OLOADTOP",
         "OLOADBOT", "CI", "IRSTTOP", "IRSTBOT", "ORSTTOP", "ORSTBOT"]     # low 12 levels, then 4 resets at the edge


def design_cfgs(net):
    out = []
    for line in open(net):
        if not line.startswith("mac "):
            continue
        kv = dict(re.findall(r"(\w+)=(\d+)", line.split(":")[0]))
        c = {k: int(kv[k]) for k in ONE}
        for k in TWO:
            c[k] = int(kv[k + "_0"]) | int(kv[k + "_1"]) << 1
        if c not in out:
            out.append(c)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("net")
    ap.add_argument("--random", type=int, default=60)
    ap.add_argument("--vectors", type=int, default=4000)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rnd = random.Random(a.seed)
    cfgs = design_cfgs(a.net)
    nd = len(cfgs)
    for _ in range(a.random):
        c = {k: rnd.randint(0, 1) for k in ONE}
        c.update({k: rnd.randint(0, 3) for k in TWO})
        cfgs.append(c)
    d = tempfile.mkdtemp(prefix="mactest")
    with open(d + "/vec", "w") as f:
        for i in range(a.vectors):
            v = [rnd.getrandbits(16) for _ in range(4)]
            if rnd.random() < .25:      # the corners where sign extension shows
                v = [rnd.choice((0, 1, 0x7F, 0x80, 0xFF, 0x7FFF, 0x8000, 0xFFFF)) for _ in range(4)]
            fl = 0
            for k, n in enumerate(FLAGS):
                p = .03 if k >= 12 else .85 if k == 0 else .15 if k < 7 else .5
                fl |= (rnd.random() < p) << k
            pulse = rnd.getrandbits(4) if rnd.random() < .05 else 0
            if i == 0:
                fl, pulse = 0xF001, 0
            f.write("%04x %04x %04x %04x %04x %x\n" % (*v, fl, pulse))
    tb = ["`timescale 1ns/1ps", "module tb;", "reg clk=0; reg [15:0] A=0,B=0,C=0,D=0,fl=0; reg [3:0] rs=0, pu; integer f,o,i,r;"]
    tb.append("wire ce=fl[0],ah=fl[1],bh=fl[2],ch=fl[3],dh=fl[4],ot=fl[5],ob=fl[6],at=fl[7],ab=fl[8],lt=fl[9],lb=fl[10],ci=fl[11];")
    for i, c in enumerate(cfgs):
        tb.append("wire [31:0] o%d; wire c%d;" % (i, i))
        par = ",".join(".%s(%d)" % (k, v) for k, v in c.items())
        tb.append("SB_MAC16 #(%s) m%d(.CLK(clk),.CE(ce),.A(A),.B(B),.C(C),.D(D),.AHOLD(ah),.BHOLD(bh),.CHOLD(ch),.DHOLD(dh),"
                  ".IRSTTOP(rs[0]),.IRSTBOT(rs[1]),.ORSTTOP(rs[2]),.ORSTBOT(rs[3]),.OLOADTOP(lt),.OLOADBOT(lb),.ADDSUBTOP(at),"
                  ".ADDSUBBOT(ab),.OHOLDTOP(ot),.OHOLDBOT(ob),.CI(ci),.ACCUMCI(1'b0),.SIGNEXTIN(1'b0),.O(o%d),.CO(c%d));" % (par, i, i, i))
    dump = "".join('$fdisplay(o,"%%h %%b",o%d,c%d);' % (i, i) for i in range(len(cfgs)))
    tb.append("task dump; begin %s end endtask" % dump)
    tb.append('initial begin f=$fopen("%s/vec","r"); o=$fopen("%s/ref","w");' % (d, d))
    tb.append('while (!$feof(f)) begin r=$fscanf(f,"%h %h %h %h %h %h\\n",A,B,C,D,fl,pu);'
              "rs=pu; #1; rs=fl[15:12]; #1; dump; clk=1; #1; dump; clk=0; #1; end $finish; end endmodule")
    open(d + "/tb.v", "w").write("\n".join(tb) + "\n")
    subprocess.check_call(["iverilog", "-g2012", "-o", d + "/tb.vvp", d + "/tb.v", CELLS])
    subprocess.check_call(["vvp", "-n", d + "/tb.vvp"], stdout=subprocess.DEVNULL)
    with open(d + "/cfg", "w") as f:
        for c in cfgs:
            f.write(" ".join(str(c[k]) for k in ORDER) + "\n")
    open(d + "/bitsim_pins.h", "w").write("#define BS_NPINS 1\n")
    subprocess.check_call(["cc", "-O1", "-w", "-I" + d, "-I" + HERE + "/..", "-DNCFG=%d" % len(cfgs), "-o", d + "/t", HERE + "/mactest.c"])
    r = subprocess.run([d + "/t", d + "/cfg", d + "/vec", d + "/ref", str(nd)], capture_output=True, text=True)
    print(r.stdout[-1800:] + r.stderr[-300:], end="")
    sys.exit(r.returncode)


if __name__ == "__main__":
    main()
