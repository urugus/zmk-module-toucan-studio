#pragma once

#include <stdint.h>

/* Multiplier x100 (A03/A05). */
#define TSP_PTR_SPEED_MIN 25
#define TSP_PTR_SPEED_MAX 800

struct tsp_ptr_speed_state {
    uint16_t cur;
    uint16_t def;
    uint16_t min;
    uint16_t max;
    /* 0 when nothing is saved. */
    uint16_t saved;
};

void tsp_ptr_speed_get(struct tsp_ptr_speed_state *out);
/* Applies immediately; RAM only. -EINVAL when out of range. */
int tsp_ptr_speed_set(uint16_t value);
/* Writes the current value to flash now. */
int tsp_ptr_speed_save(void);
/* Back to the saved value, or the default when nothing is saved. */
void tsp_ptr_speed_revert(void);
/* Deletes the saved value and applies the default. */
int tsp_ptr_speed_reset(void);
