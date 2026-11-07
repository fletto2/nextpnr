/* cell models, included into the generated file; config arguments fold to constants */
#define DELTA_MAX 256
#define LOOP_MAX 64

struct bitsim_stats bs_stats;
static uint8_t ext_en[NEXT], ext_val[NEXT];

static uint16_t spram[NSPRAM ? NSPRAM : 1][16384];
static uint16_t spram_out[NSPRAM ? NSPRAM : 1];

/* SB_RAM40_4K */
static uint16_t ram[NRAM ? NRAM : 1][256];
static uint16_t ram_rd[NRAM ? NRAM : 1];

static inline uint16_t ram_lanes(int mode, unsigned addr)
{
	switch (mode) {
	case 1: return addr >> 8 & 1 ? 0x5555 : 0xAAAA;
	case 2: return (uint16_t)(0x1111u << (addr >> 8 & 3) ^ 0xFFFF);
	case 3: return (uint16_t)(0x0101u << (addr >> 8 & 7) ^ 0xFFFF);
	default: return 0;
	}
}
#define BIT(x, n) ((x) >> (n) & 1u)
static inline void ram_read(int i, int mode, unsigned addr, int cflag)
{
	uint16_t d = ram[i][addr & 255] & (uint16_t)~ram_lanes(mode, addr);
	if (d != ram_rd[i]) {
		ram_rd[i] = d;
		F[cflag] = 1;
	}
}
static inline void ram_write(int i, int mode, unsigned addr, unsigned mask, unsigned d)
{
	unsigned wm = mode ? ram_lanes(mode, addr) : mask, x = 0, k;
	switch (mode) {
	case 0: x = d; break;
	case 1: for (k = 0; k < 16; k += 2) x |= BIT(d, k) * 3u << k; break;
	case 2: for (k = 0; k < 16; k += 4) x |= BIT(d, k + 1) * 15u << k; break;
	case 3: x = BIT(d, 3) * 0xFFu | BIT(d, 11) * 0xFF00u; break;
	}
	ram[i][addr & 255] = (uint16_t)((ram[i][addr & 255] & wm) | (x & ~wm));
}
static inline unsigned ram_out(int mode, int i)
{
	unsigned r = ram_rd[i], o = 0, k;
	switch (mode) {
	case 0: return r;
	case 1: for (k = 0; k < 16; k += 2) o |= (unsigned)!!(r >> k & 3) << k; return o;
	case 2: for (k = 0; k < 4; k++) o |= (unsigned)!!(r >> (4 * k) & 15) << (4 * k + 1); return o;
	default: return (unsigned)!!(r & 0xFF) << 3 | (unsigned)!!(r & 0xFF00) << 11;
	}
}

/* SB_SPRAM256KA. x in the cell model becomes "keep last value" */
static inline void spram_clk(int i, unsigned cs, unsigned wren, unsigned standby, unsigned sleep, unsigned pwr,
			     unsigned addr, unsigned d, unsigned mw, int cflag)
{
	unsigned m, w;
	if (sleep || !pwr) {
		spram_out[i] = 0;
		F[cflag] = 1;
		return;
	}
	if (standby || !cs)
		return;
	if (!wren) {
		if (spram_out[i] != spram[i][addr]) {
			spram_out[i] = spram[i][addr];
			F[cflag] = 1;
		}
		return;
	}
	m = (mw & 1 ? 0xFu : 0) | (mw & 2 ? 0xF0u : 0) | (mw & 4 ? 0xF00u : 0) | (mw & 8 ? 0xF000u : 0);
	w = spram[i][addr];
	spram[i][addr] = (uint16_t)((w & ~m) | (d & m));
}

/* SB_MAC16 */
typedef struct {
	uint16_t rA, rB, rC, rD, rF, rJ, rK, rG, rQ, rS;
	uint32_t rH;
} mac_t;
typedef struct {
	uint32_t o, iL;
	uint16_t pF, pJ, pK, pG, iP, iR;
	unsigned co;
} mac_res;
static mac_t mac[NMAC];

static inline __attribute__((always_inline)) void
mac_comb(int a_reg, int b_reg, int c_reg, int d_reg, int top8, int bot8, int p1, int p2,
	 int top_out, int top_low, int top_up, int top_cs, int bot_out, int bot_low, int bot_up, int bot_cs,
	 int mode8, int a_s, int b_s, const mac_t *m, mac_res *r, unsigned A, unsigned B, unsigned C, unsigned Dd,
	 unsigned addsubtop, unsigned addsubbot, unsigned oloadtop, unsigned oloadbot, unsigned ci)
{
	unsigned iA = a_reg ? m->rA : A, iB = b_reg ? m->rB : B, iC = c_reg ? m->rC : C, iD = d_reg ? m->rD : Dd;
	unsigned Ah = (iA >> 8) | (a_s && BIT(iA, 15) ? 0xFF00u : 0);
	unsigned Al = (iA & 0xFF) | (a_s && mode8 && BIT(iA, 7) ? 0xFF00u : 0);
	unsigned Bh = (iB >> 8) | (b_s && BIT(iB, 15) ? 0xFF00u : 0);
	unsigned Bl = (iB & 0xFF) | (b_s && mode8 && BIT(iB, 7) ? 0xFF00u : 0);
	unsigned pF = (Ah * Bh) & 0xFFFF, pJ = ((Al & 0xFF) * Bh) & 0xFFFF;
	unsigned pK = (Ah * (Bl & 0xFF)) & 0xFFFF, pG = (Al * Bl) & 0xFFFF;
	unsigned iF = top8 ? m->rF : pF, iJ = p1 ? m->rJ : pJ, iK = p1 ? m->rK : pK, iG = bot8 ? m->rG : pG;
	uint32_t iK_e = iK | (a_s && BIT(iK, 15) ? 0xFF0000u : 0);
	uint32_t iJ_e = iJ | (b_s && BIT(iJ, 15) ? 0xFF0000u : 0);
	uint32_t iL = iG + (iK_e << 8) + (iJ_e << 8) + ((uint32_t)iF << 16);
	uint32_t iH = p2 ? m->rH : iL;
	unsigned iS = m->rS, iQ = m->rQ, iY, iZ, LCI, yz, LCO, YZ, iR, HCI, iW, iX, xw, ACO, XW, iP, Oh, Ol;

	iY = bot_up ? iD : iS;
	iZ = bot_low == 0 ? iB : bot_low == 1 ? iG : bot_low == 2 ? (iH & 0xFFFF) : 0;
	LCI = bot_cs == 0 ? 0 : bot_cs == 1 ? 1 : bot_cs == 2 ? 0 : ci;
	yz = iZ + ((iY ^ (addsubbot ? 0xFFFFu : 0)) & 0xFFFF) + LCI;
	LCO = yz >> 16 & 1;
	YZ = yz & 0xFFFF;
	iR = oloadbot ? iD : (YZ ^ (addsubbot ? 0xFFFFu : 0));

	HCI = top_cs == 0 ? 0 : top_cs == 1 ? 1 : top_cs == 2 ? LCO : LCO ^ addsubbot;
	iW = top_up ? iC : iQ;
	iX = top_low == 0 ? iA : top_low == 1 ? iF : top_low == 2 ? (iH >> 16) : (BIT(iZ, 15) ? 0xFFFFu : 0);
	xw = iX + ((iW ^ (addsubtop ? 0xFFFFu : 0)) & 0xFFFF) + HCI;
	ACO = xw >> 16 & 1;
	XW = xw & 0xFFFF;
	iP = oloadtop ? iC : (XW ^ (addsubtop ? 0xFFFFu : 0));

	Oh = top_out == 0 ? iP : top_out == 1 ? iQ : top_out == 2 ? iF : (iH >> 16);
	Ol = bot_out == 0 ? iR : bot_out == 1 ? iS : bot_out == 2 ? iG : (iH & 0xFFFF);
	r->o = (uint32_t)Oh << 16 | Ol;
	r->co = ACO ^ addsubtop;
	r->iL = iL;
	r->pF = (uint16_t)pF;
	r->pJ = (uint16_t)pJ;
	r->pK = (uint16_t)pK;
	r->pG = (uint16_t)pG;
	r->iP = (uint16_t)iP;
	r->iR = (uint16_t)iR;
}

static inline void mac_rst_irsttop(mac_t *m) { m->rC = m->rA = m->rF = m->rJ = 0; }
static inline void mac_rst_irstbot(mac_t *m) { m->rB = m->rD = m->rK = m->rG = 0; m->rH = 0; }
static inline void mac_rst_orsttop(mac_t *m) { m->rQ = 0; }
static inline void mac_rst_orstbot(mac_t *m) { m->rS = 0; }

static inline __attribute__((always_inline)) void
mac_step(int mode8, mac_t *m, const mac_res *r, unsigned A, unsigned B, unsigned C, unsigned Dd, unsigned ce,
	 unsigned ahold, unsigned bhold, unsigned chold, unsigned dhold, unsigned ohtop, unsigned ohbot,
	 unsigned irsttop, unsigned irstbot, unsigned orsttop, unsigned orstbot)
{
	if (irsttop) {
		mac_rst_irsttop(m);
	} else if (ce) {
		if (!chold) m->rC = (uint16_t)C;
		if (!ahold) m->rA = (uint16_t)A;
		m->rF = r->pF;
		if (!mode8) m->rJ = r->pJ;
	}
	if (irstbot) {
		mac_rst_irstbot(m);
	} else if (ce) {
		if (!bhold) m->rB = (uint16_t)B;
		if (!dhold) m->rD = (uint16_t)Dd;
		m->rG = r->pG;
		if (!mode8) {
			m->rK = r->pK;
			m->rH = r->iL;
		}
	}
	if (orsttop) mac_rst_orsttop(m);
	else if (ce && !ohtop) m->rQ = r->iP;
	if (orstbot) mac_rst_orstbot(m);
	else if (ce && !ohbot) m->rS = r->iR;
}
