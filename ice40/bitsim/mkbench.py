#!/usr/bin/env python3
"""mkbench.py SRC_TB OUT_TB: gate-only recording copy of a cosim bench.
Drops the RTL side, logs every input pad change and samples all output pads
1 ns after each clk25 edge. Also adds bus reads, which the stock benches never do."""
import re, sys

src, dst = sys.argv[1:3]
s = open(src).read()
scene = 'gold.vram_if' in s

s = re.sub(r'\n\s*top gold\(.*?\);', '\n', s, count=1, flags=re.S)
s = re.sub(r'\n\s*wire \[28:0\] og = .*?(\n\s*task bus_write)', r'''
    integer hard = 0, xerr = 0, err = 0, known = 0, k, reads = 0, cutoff = 0, lastt = 0;
    integer fo, fe, nrd = 8;
    reg [1023:0] outf, evf, memf;
    function [63:0] ps(input real t); ps = t * 1000 + 0.5; endfunction
    initial begin
        if (!$value$plusargs("out=%s", outf)) outf = "out.txt";
        if (!$value$plusargs("ev=%s", evf)) evf = "ev.txt";
        if (!$value$plusargs("reads=%d", nrd)) nrd = 8;
        fo = $fopen(outf); fe = $fopen(evf);
    end
    always @(clk25) begin
        #1 $fdisplay(fo, "%0d %b%b%b%b%b%b%b%b%b%b%b%b%b%b", ps($realtime), irq_n, r_n, g_n, b_n, hs_n, vs_n,
                     sck_n, mosi_n, ss_n, lr_n, bck_n, ad_n, d_n, 1'b0);
    end
    always @(cs_n, rd_n, wr_n, a, dout, drive, miso)
        $fdisplay(fe, "%0d %b%b%b %b %b %b %b", ps($realtime), cs_n, rd_n, wr_n, a, dout, drive, miso);
\1''', s, count=1, flags=re.S)
s = re.sub(r'(#\(\$urandom % 60 \+ 80\);\n\s*reads = reads \+ 1;\n).*?(\n\s*rd_n = 1;)', r'\1\2', s, flags=re.S)
s = re.sub(r'\n\s*gate\.\\vram_if\.main_ram\.blk(\d)_RAM \.u\.mem\[m\] = w\d;[^\n]*',
           lambda m: '\n            gate.\\vram_if.main_ram.blk%s_RAM .u.mem[m] = w%s;' % (m[1], m[1]), s)
if scene:
    s = s.replace('        #20000;\n        // sprite table', '''        if ($value$plusargs("mem=%s", memf)) begin
            $writememh({memf, "0"}, gate.\\vram_if.main_ram.blk0_RAM .u.mem);
            $writememh({memf, "1"}, gate.\\vram_if.main_ram.blk1_RAM .u.mem);
            $writememh({memf, "2"}, gate.\\vram_if.main_ram.blk2_RAM .u.mem);
            $writememh({memf, "3"}, gate.\\vram_if.main_ram.blk3_RAM .u.mem);
        end
        #20000;
        // sprite table''', 1)
    s = s.replace('''            #32000;
''', '''            #32000;
            for (m = 0; m < nrd; m = m + 1) begin bus_read($urandom % 32); #($urandom % 40); end
            for (m = 0; m < nfx; m = m + 1) fx_round;
''', 1)
    # the stock benches never touch the FX multiplier/accumulator, the only MAC16 with its adders on
    s = s.replace('''    integer m, i, n, seed, sd, mode, nspr;
''', '''    integer nfx = 0, fxa = 0, fxi;
    task fx_round;
        begin
            wr(5, 6 << 1);
            wr(9, $urandom); wr(10, $urandom); wr(11, $urandom); wr(12, $urandom);
            wr(5, 2 << 1); wr(9, 8'h40); wr(12, ($urandom & 8'hE0) | 8'h10);
            wr(5, 6 << 1);
            for (fxi = $urandom % 4; fxi > 0; fxi = fxi - 1) bus_read(10);
            if ($urandom % 4 == 0) bus_read(9);
            fxa = 17'h10000 + 4 * ($urandom % 1024);
            seta(fxa); wr(3, 0);
            wr(5, 2 << 1); wr(9, 0); wr(12, 0);
            seta(fxa);
            for (fxi = 0; fxi < 4; fxi = fxi + 1) bus_read(3);
        end
    endtask
    integer m, i, n, seed, sd, mode, nspr;
''', 1)
    s = s.replace('''        if (!$value$plusargs("lines=%d", n)) n = 200;
''', '''        if (!$value$plusargs("lines=%d", n)) n = 200;
        if (!$value$plusargs("fx=%d", nfx)) nfx = 0;
''', 1)
else:
    # no preload in this bench: fill SPRAM with the power-up zeros bitsim assumes, or video stays x
    s = s.replace('        #20000;\n        // PSG', '''        for (i = 0; i < 16384; i = i + 1) begin
            gate.\\vram_if.main_ram.blk0_RAM .u.mem[i] = 0; gate.\\vram_if.main_ram.blk1_RAM .u.mem[i] = 0;
            gate.\\vram_if.main_ram.blk2_RAM .u.mem[i] = 0; gate.\\vram_if.main_ram.blk3_RAM .u.mem[i] = 0;
        end
        #20000;
        // PSG''', 1)
    s = s.replace('''            #20480;                                     // one sample period (512 clk25)
''', '''            #20480;                                     // one sample period (512 clk25)
            for (j = 0; j < nrd; j = j + 1) begin bus_read($urandom % 32); #($urandom % 40); end
''', 1)
open(dst, 'w').write(s)
