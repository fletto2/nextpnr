/* driven by ramtest.py: instance i is read mode i>>2, write mode i&3 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "bitsim.h"
#define NRAM 16
#define NSPRAM 1
#define NMAC 1
#define NEXT 1
static uint8_t F[8];
#include "runtime.inc"

int main(int argc, char **argv)
{
	FILE *fv = fopen(argv[1], "r"), *fr = fopen(argv[2], "r");
	unsigned ra, wa, mk, wd, fl, step = 0;
	long total = 0, errs = 0;
	int bad[16] = { 0 };

	(void)argc;
	while (fscanf(fv, "%x %x %x %x %x", &ra, &wa, &mk, &wd, &fl) == 5) {
		for (int i = 0; i < 16; i++) {
			unsigned ref, q;
			/* same order as the generated edge: reads see the old contents */
			if ((fl & 3) == 3) ram_read(i, i >> 2, ra, 0);
			if ((fl & 12) == 12) ram_write(i, i & 3, wa, mk, wd);
			q = ram_out(i >> 2, i);
			if (fscanf(fr, "%x", &ref) != 1) return 3;
			total++;
			if (ref != q) {
				if (!bad[i]++)
					printf("read mode %d write mode %d step %u: ref %04x model %04x\n", i >> 2, i & 3, step, ref, q);
				errs++;
			}
		}
		step++;
	}
	printf("%u steps, %ld compares, %ld mismatches\n", step, total, errs);
	return errs != 0;
}
