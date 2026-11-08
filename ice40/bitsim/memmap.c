/* memmap: which SPRAM cell holds which RTL bank, read out of the image itself.
 * blk0/blk1 hold VRAM word addresses with bit 14 clear, blk2/blk3 with it set; blk0/blk2 carry data
 * bits 15:0, blk1/blk3 bits 31:16. One byte written over the bus to each bank lands in exactly one
 * cell, and that cell is the bank's. Prints the order for replay's --mem and tgen's --mem-order. */
#include <stdio.h>
#include <stdint.h>
#include "bitsim.h"

static const int dpin[8] = { BS_extbus_d_0, BS_extbus_d_1, BS_extbus_d_2, BS_extbus_d_3,
	BS_extbus_d_4, BS_extbus_d_5, BS_extbus_d_6, BS_extbus_d_7 };
static const int apin[5] = { BS_extbus_a_0, BS_extbus_a_1, BS_extbus_a_2, BS_extbus_a_3, BS_extbus_a_4 };
static int64_t t, clk_next = 10500;
static int clk;

/* run to time `to`, clk50 toggling every 10 ns as in the traces */
static int run(int64_t to)
{
	while (clk_next <= to) {
		t = clk_next;
		clk ^= 1;
		bitsim_set(BS_clk50, clk);
		clk_next += 10000;
		if (bitsim_eval() < 0) return -1;
	}
	t = to;
	return bitsim_eval() < 0 ? -1 : 0;
}

/* one 12 MHz style write cycle: cs, then wr low 64 ns, data held across it */
static int wr(int a, int d)
{
	int i, r = 0;
	for (i = 0; i < 5; i++) bitsim_set(apin[i], a >> i & 1);
	for (i = 0; i < 8; i++) bitsim_set(dpin[i], d >> i & 1);
	bitsim_set(BS_extbus_cs_n, 0);
	r |= run(t + 10000);
	bitsim_set(BS_extbus_wr_n, 0);
	r |= run(t + 64000);
	bitsim_set(BS_extbus_wr_n, 1);
	r |= run(t + 10000);
	bitsim_set(BS_extbus_cs_n, 1);
	for (i = 0; i < 8; i++) bitsim_release(dpin[i]);
	r |= run(t + 300000);
	return r;
}

int main(void)
{
	/* byte addresses of word 0 in each bank, bank = (bit 16, bit 1) */
	static const int addr[4] = { 0x00000, 0x00002, 0x10000, 0x10002 };
	int cell[4], k, z, r = 0;

	bitsim_reset();
	bitsim_set(BS_extbus_cs_n, 1);
	bitsim_set(BS_extbus_rd_n, 1);
	bitsim_set(BS_extbus_wr_n, 1);
	bitsim_set(BS_pll_lock, 1);
	r |= run(20000000);     /* the traces start the CPU at 20 us; reset is long over */
	for (k = 0; k < 4; k++) {
		r |= wr(0, addr[k] & 255);
		r |= wr(1, addr[k] >> 8 & 255);
		r |= wr(2, addr[k] >> 16 & 1);
		r |= wr(3, 0xA1 + k);
	}
	r |= run(t + 2000000);
	if (r) { fprintf(stderr, "memmap: no convergence\n"); return 1; }
	for (k = 0; k < 4; k++) {
		cell[k] = 0;
		for (z = 1; z <= 4; z++) {
			int w = bitsim_spram_peek(z, 0);
			if (w >= 0 && ((w & 255) == 0xA1 + k || (w >> 8) == 0xA1 + k)) {
				if (cell[k]) { fprintf(stderr, "memmap: bank %d in two cells\n", k); return 1; }
				cell[k] = z;
			}
		}
		if (!cell[k]) { fprintf(stderr, "memmap: bank %d write landed nowhere\n", k); return 1; }
	}
	printf("%d,%d,%d,%d\n", cell[0], cell[1], cell[2], cell[3]);
	return 0;
}
