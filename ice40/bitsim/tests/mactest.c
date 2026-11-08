/* driven by mactest.py: the MAC model, step for step against the reference dump */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bitsim.h"
#define NRAM 1
#define NSPRAM 1
#define NMAC NCFG
#define NEXT 1
static uint8_t F[8];
#include "runtime.inc"

static int cf[NCFG][19];
static mac_res res;
static void eval(int i, unsigned A, unsigned B, unsigned C, unsigned D, unsigned fl)
{
	int *c = cf[i];
	mac_comb(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9], c[10], c[11], c[12], c[13], c[14], c[15],
		 c[16], c[17], c[18], &mac[i], &res, A, B, C, D, fl >> 7 & 1, fl >> 8 & 1, fl >> 9 & 1, fl >> 10 & 1, fl >> 11 & 1);
}

int main(int argc, char **argv)
{
	FILE *fc = fopen(argv[1], "r"), *fv = fopen(argv[2], "r"), *fr = fopen(argv[3], "r");
	int nd = argc > 4 ? atoi(argv[4]) : 0, bad[NCFG], step = 0;
	unsigned A, B, C, D, fl, pu;
	long total = 0, errs = 0;

	memset(bad, 0, sizeof bad);
	for (int i = 0; i < NCFG; i++)
		for (int k = 0; k < 19; k++)
			if (fscanf(fc, "%d", &cf[i][k]) != 1) return 2;
	while (fscanf(fv, "%x %x %x %x %x %x", &A, &B, &C, &D, &fl, &pu) == 6) {
		unsigned rs = fl >> 12;
		for (int ph = 0; ph < 2; ph++) {
			for (int i = 0; i < NCFG; i++) {
				char o[40], co[8];
				mac_t *m = &mac[i];
				if (!ph) {
					unsigned e = pu | rs;   /* both levels reach the cell before the edge */
					if (e & 1) mac_rst_irsttop(m);
					if (e & 2) mac_rst_irstbot(m);
					if (e & 4) mac_rst_orsttop(m);
					if (e & 8) mac_rst_orstbot(m);
				}
				eval(i, A, B, C, D, fl);
				if (ph) {
					/* the edge happened between dump 0 and dump 1: replay it from the pre-edge state */
				}
				if (fscanf(fr, "%39s %7s", o, co) != 2) return 3;
				char want[40];
				snprintf(want, sizeof want, "%08x", res.o);
				total++;
				if (strcmp(want, o) || (unsigned)(co[0] - '0') != res.co) {
					if (!bad[i]++ )
						printf("cfg %d%s step %d ph %d: ref %s co %s, model %s co %u\n", i, i < nd ? " (design)" : "", step, ph, o, co, want, res.co);
					errs++;
				}
				if (!ph)
					mac_step(cf[i][16], m, &res, A, B, C, D, fl & 1, fl >> 1 & 1, fl >> 2 & 1, fl >> 3 & 1, fl >> 4 & 1,
						 fl >> 5 & 1, fl >> 6 & 1, rs & 1, rs >> 1 & 1, rs >> 2 & 1, rs >> 3 & 1);
			}
		}
		step++;
	}
	int nb = 0, nbd = 0;
	for (int i = 0; i < NCFG; i++) { nb += !!bad[i]; nbd += i < nd && bad[i]; }
	printf("%d configs (%d design), %d steps, %ld compares, %ld mismatches, bad configs %d (design %d)\n", NCFG, nd, step, total, errs, nb, nbd);
	return errs != 0;
}
