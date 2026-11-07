#ifndef BITSIM_H
#define BITSIM_H
#include <stdint.h>
#include "bitsim_pins.h"

struct bitsim_stats {
	uint64_t evals, deltas, delta_fail, loop_fail;
};
extern struct bitsim_stats bs_stats;
extern const char *const bitsim_pin_names[BS_NPINS];

void bitsim_reset(void);                /* power-up: FFs, SPRAM 0, BRAM from INIT */
void bitsim_set(int pin, int val);      /* drive; on a bidir pad this turns the external driver on */
void bitsim_release(int pin);           /* external driver off */
int  bitsim_eval(void);                 /* run to stable; delta count, -1 if it did not converge */
int  bitsim_get(int pin);               /* pad value after the last eval */
int  bitsim_driven(int pin);            /* the FPGA drives this pad */
int  bitsim_contention(void);           /* bidir pads driven from both sides */
void bitsim_spram_load(int cell, const uint16_t *words, int n);   /* cell = chipdb z, 1..4 */
#endif
