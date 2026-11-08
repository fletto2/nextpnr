#!/usr/bin/env python3
"""ramtest.py [--vectors N] [--seed N]: runtime.inc's block RAM against yosys' SB_RAM40_4K, all 16 mode pairs."""
import argparse, os, random, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CELLS = "/opt/oss-fpga/share/yosys/ice40/cells_sim.v"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vectors", type=int, default=40000)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rnd = random.Random(a.seed)
    d = tempfile.mkdtemp(prefix="ramtest")
    with open(d + "/vec", "w") as f:
        for _ in range(a.vectors):
            # few rows, every lane select: reads meet writes often
            ra = rnd.getrandbits(3) << 8 | rnd.getrandbits(3) if rnd.random() < .8 else rnd.getrandbits(11)
            wa = rnd.getrandbits(3) << 8 | rnd.getrandbits(3) if rnd.random() < .8 else rnd.getrandbits(11)
            mask = rnd.getrandbits(16) if rnd.random() < .5 else 0
            fl = (rnd.random() < .7) | (rnd.random() < .9) << 1 | (rnd.random() < .5) << 2 | (rnd.random() < .9) << 3
            f.write("%03x %03x %04x %04x %x\n" % (ra, wa, mask, rnd.getrandbits(16), fl))
    tb = ["`timescale 1ns/1ps", "module tb; reg clk=0; reg [10:0] ra=0,wa=0; reg [15:0] mk=0,wd=0; reg [3:0] fl=0; integer f,o,r;"]
    for i in range(16):
        tb.append("wire [15:0] q%d; SB_RAM40_4K #(.READ_MODE(%d),.WRITE_MODE(%d)) m%d(.RDATA(q%d),.RCLK(clk),.RCLKE(fl[1]),.RE(fl[0]),"
                  ".RADDR(ra),.WCLK(clk),.WCLKE(fl[3]),.WE(fl[2]),.WADDR(wa),.MASK(mk),.WDATA(wd));" % (i, i >> 2, i & 3, i, i))
    dump = "".join('$fdisplay(o,"%%h",q%d);' % i for i in range(16))
    tb.append('initial begin f=$fopen("%s/vec","r"); o=$fopen("%s/ref","w");' % (d, d))
    tb.append('while (!$feof(f)) begin r=$fscanf(f,"%%h %%h %%h %%h %%h\\n",ra,wa,mk,wd,fl); #1; clk=1; #1; %s clk=0; #1; end $finish; end endmodule' % dump)
    open(d + "/tb.v", "w").write("\n".join(tb) + "\n")
    subprocess.check_call(["iverilog", "-g2012", "-o", d + "/tb.vvp", d + "/tb.v", CELLS])
    subprocess.check_call(["vvp", "-n", d + "/tb.vvp"], stdout=subprocess.DEVNULL)
    open(d + "/bitsim_pins.h", "w").write("#define BS_NPINS 1\n")
    subprocess.check_call(["cc", "-O1", "-w", "-I" + d, "-I" + HERE + "/..", "-o", d + "/t", HERE + "/ramtest.c"])
    r = subprocess.run([d + "/t", d + "/vec", d + "/ref"], capture_output=True, text=True)
    print(r.stdout[-1500:] + r.stderr[-300:], end="")
    sys.exit(r.returncode)


if __name__ == "__main__":
    main()
