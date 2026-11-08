/* replay.c: drive bitsim from a recorded bench, compare every output pad sample.
 * replay EV OUT --clk50 FIRST_TOGGLE_PS [--mem PREFIX Z0,Z1,Z2,Z3] [--maxerr N] [--nocheck] */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "bitsim.h"

#ifdef BS_COUNT
void bitsim_counts(const char *path);
int bitsim_vsnap(const uint8_t **val, const int **nets);
#endif

struct ev { int64_t t; uint8_t cs, rd, wr, a, d, drive, miso; };
struct smp { int64_t t; char b[32]; };

/* column order of the bench's output line */
static const int col[30] = {
	BS_extbus_irq_n,
	BS_vga_r_3, BS_vga_r_2, BS_vga_r_1, BS_vga_r_0,
	BS_vga_g_3, BS_vga_g_2, BS_vga_g_1, BS_vga_g_0,
	BS_vga_b_3, BS_vga_b_2, BS_vga_b_1, BS_vga_b_0,
	BS_vga_hsync, BS_vga_vsync, BS_spi_sck, BS_spi_mosi, BS_spi_ssel_n_sd,
	BS_audio_lrck, BS_audio_bck, BS_audio_data,
	BS_extbus_d_7, BS_extbus_d_6, BS_extbus_d_5, BS_extbus_d_4,
	BS_extbus_d_3, BS_extbus_d_2, BS_extbus_d_1, BS_extbus_d_0, -1,
};
static const int dpin[8] = { BS_extbus_d_0, BS_extbus_d_1, BS_extbus_d_2, BS_extbus_d_3,
			     BS_extbus_d_4, BS_extbus_d_5, BS_extbus_d_6, BS_extbus_d_7 };
static const int apin[5] = { BS_extbus_a_0, BS_extbus_a_1, BS_extbus_a_2, BS_extbus_a_3, BS_extbus_a_4 };

/* the first recordings kept time in a 32-bit integer: times only ever go up, so unwrap */
static int64_t unwrap(int64_t t, int64_t last)
{
	while (t < last)
		t += INT64_C(1) << 32;
	return t;
}

static unsigned bin(const char *s)
{
	unsigned v = 0;
	for (; *s == '0' || *s == '1'; s++)
		v = v << 1 | (unsigned)(*s - '0');
	return v;
}

static void load_mem(const char *path, int z)
{
	static uint16_t w[16384];
	char line[256];
	int n = 0;
	FILE *f = fopen(path, "r");
	if (!f) { perror(path); exit(2); }
	while (fgets(line, sizeof line, f)) {
		char *p = line;
		while (*p == ' ') p++;
		if (*p == '/' || *p == '\n' || *p == '@' || !*p)
			continue;
		if (n < 16384)
			w[n++] = (uint16_t)strtoul(p, NULL, 16);
	}
	fclose(f);
	if (n != 16384) { fprintf(stderr, "%s: %d words\n", path, n); exit(2); }
	bitsim_spram_load(z, w, n);
}

static void set_bus(const struct ev *e, const struct ev *o)
{
	int i;
	if (e->cs != o->cs) bitsim_set(BS_extbus_cs_n, e->cs);
	if (e->rd != o->rd) bitsim_set(BS_extbus_rd_n, e->rd);
	if (e->wr != o->wr) bitsim_set(BS_extbus_wr_n, e->wr);
	if (e->miso != o->miso) bitsim_set(BS_spi_miso, e->miso);
	for (i = 0; i < 5; i++)
		if ((e->a ^ o->a) >> i & 1)
			bitsim_set(apin[i], e->a >> i & 1);
	if (e->drive != o->drive || e->d != o->d)
		for (i = 0; i < 8; i++) {
			if (e->drive) bitsim_set(dpin[i], e->d >> i & 1);
			else bitsim_release(dpin[i]);
		}
}

static double now(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv)
{
	struct ev *ev;
	struct smp *sm;
	size_t nev = 0, nsm = 0, cap;
	FILE *f;
	char line[256], a[16], b[16], c[16], d[16], g[16];
	long long t;
	int64_t last, clk_next = -1, tt;
	int maxerr = 20, clk = 0, check = 1;
	const char *mem = NULL, *memz = NULL;
	uint64_t compared = 0, skip_x = 0, skip_z = 0, errs = 0, contention = 0, edges = 0;
	double t0, t1;
	size_t ei = 0, si = 0;
	struct ev cur = { 0, 1, 1, 1, 0, 0, 0, 0 };

	if (argc < 3) { fprintf(stderr, "usage: replay EV OUT --clk50 PS [--mem PFX Z,Z,Z,Z] [--maxerr N] [--nocheck]\n"); return 2; }
	for (int k = 3; k < argc; k++) {
		if (!strcmp(argv[k], "--clk50")) clk_next = atoll(argv[++k]);
		else if (!strcmp(argv[k], "--mem")) { mem = argv[++k]; memz = argv[++k]; }
		else if (!strcmp(argv[k], "--maxerr")) maxerr = atoi(argv[++k]);
		else if (!strcmp(argv[k], "--nocheck")) check = 0;
	}
	if (clk_next < 0) { fprintf(stderr, "--clk50 needed\n"); return 2; }

	f = fopen(argv[1], "r");
	if (!f) { perror(argv[1]); return 2; }
	cap = 1 << 16;
	ev = malloc(cap * sizeof *ev);
	last = 0;
	while (fgets(line, sizeof line, f)) {
		if (sscanf(line, "%lld %15s %15s %15s %15s %15s", &t, a, b, c, d, g) != 6)
			continue;
		if (nev == cap) ev = realloc(ev, (cap *= 2) * sizeof *ev);
		last = ev[nev].t = unwrap(t, last);
		ev[nev].cs = a[0] == '1'; ev[nev].rd = a[1] == '1'; ev[nev].wr = a[2] == '1';
		ev[nev].a = (uint8_t)bin(b); ev[nev].d = (uint8_t)bin(c);
		ev[nev].drive = d[0] == '1'; ev[nev].miso = g[0] == '1';
		nev++;
	}
	fclose(f);
	f = fopen(argv[2], "r");
	if (!f) { perror(argv[2]); return 2; }
	cap = 1 << 18;
	sm = malloc(cap * sizeof *sm);
	last = 0;
	while (fgets(line, sizeof line, f)) {
		char s[40];
		if (sscanf(line, "%lld %39s", &t, s) != 2 || strlen(s) != 30)
			continue;       /* a torn last line while the bench is still running */
		if (nsm == cap) sm = realloc(sm, (cap *= 2) * sizeof *sm);
		last = sm[nsm].t = unwrap(t, last);
		memcpy(sm[nsm].b, s, 30);
		nsm++;
	}
	fclose(f);

	bitsim_reset();
	if (mem) {
		int z[4];
		char p[512];
		sscanf(memz, "%d,%d,%d,%d", &z[0], &z[1], &z[2], &z[3]);
		for (int k = 0; k < 4; k++) {
			snprintf(p, sizeof p, "%s%d", mem, k);
			load_mem(p, z[k]);
		}
	}
	bitsim_set(BS_extbus_cs_n, 1);
	bitsim_set(BS_extbus_rd_n, 1);
	bitsim_set(BS_extbus_wr_n, 1);
	bitsim_set(BS_pll_lock, 1);
	if (bitsim_eval() < 0) printf("t=0 did not converge\n");

	t0 = now();
	while (si < nsm) {
		int acted = 0;
		tt = sm[si].t;
		if (clk_next < tt) tt = clk_next;
		if (ei < nev && ev[ei].t < tt) tt = ev[ei].t;
		while (ei < nev && ev[ei].t == tt) {
			set_bus(&ev[ei], &cur);
			cur = ev[ei++];
			acted = 1;
		}
		if (clk_next == tt) {
			clk ^= 1;
			bitsim_set(BS_clk50, clk);
			clk_next += 10000;
			edges++;
			acted = 1;
		}
#ifdef BS_COUNT
		/* every 4th eval: which nets it changed, for gen.py --trig */
		static FILE *tf;
		static uint8_t pre[1 << 15];
		static uint64_t nevals;
		const uint8_t *vv;
		const int *vn;
		int nv = bitsim_vsnap(&vv, &vn), snap = acted && (nevals++ & 3) == 0 && nv <= (int)sizeof pre;
		if (!tf) tf = fopen("trig.txt", "w");
		if (snap) memcpy(pre, vv, nv);
#endif
		if (acted && bitsim_eval() < 0) {
			printf("%" PRId64 " ps: no convergence\n", tt);
			break;
		}
#ifdef BS_COUNT
		if (snap && tf) {
			for (int k = 0; k < nv; k++)
				if (pre[k] != vv[k]) fprintf(tf, "%d ", vn[k]);
			fputc('\n', tf);
		}
#endif
		if (acted) contention += (uint64_t)(bitsim_contention() != 0);
		if (sm[si].t == tt) {
			for (int k = 0; check && k < 29; k++) {
				char r = sm[si].b[k];
				int pin = col[k], val, drv;
				if (r == 'x' || r == 'X') { skip_x++; continue; }
				val = bitsim_get(pin);
				drv = bitsim_driven(pin) || cur.drive;
				if (k >= 21 && (r == 'z' || r == 'Z')) {
					skip_z++;
					if (drv) {
						errs++;
						if (errs <= (uint64_t)maxerr)
							printf("%" PRId64 " ps %s: ref z, sim driven\n", tt, bitsim_pin_names[pin]);
					}
					continue;
				}
				compared++;
				if (val != r - '0') {
					errs++;
					if (errs <= (uint64_t)maxerr)
						printf("%" PRId64 " ps %s: ref %c sim %d\n", tt, bitsim_pin_names[pin], r, val);
				}
			}
			si++;
		}
	}
	t1 = now();
	printf("samples %zu to %.3f ms; bits compared %" PRIu64 ", mismatches %" PRIu64 ", skipped x %" PRIu64 ", z %" PRIu64 "\n",
	       si, (double)(si ? sm[si - 1].t : 0) * 1e-9, compared, errs, skip_x, skip_z);
	printf("clk50 edges %" PRIu64 " in %.3f s: %.0f edges/s; evals %" PRIu64 ", deltas %" PRIu64 ", delta fails %" PRIu64 ", loop fails %" PRIu64 ", contention %" PRIu64 "\n",
	       edges, t1 - t0, (double)edges / (t1 - t0), bs_stats.evals, bs_stats.deltas, bs_stats.delta_fail,
	       bs_stats.loop_fail, contention);
#ifdef BS_COUNT
	bitsim_counts("act.txt");
#endif
	return errs != 0;
}
