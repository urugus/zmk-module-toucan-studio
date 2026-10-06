#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Right-half trackpad settings (M6, A14/A16). */
#define TSP_RIGHT_SENS_MIN 1 /* ADC attenuation 1x..4x */
#define TSP_RIGHT_SENS_MAX 4
/* Touch threshold z (D27): Y-axis wide z-min = z, X-axis = z + the devicetree's X - Y. */
#define TSP_RIGHT_Z_MIN 2
#define TSP_RIGHT_Z_MAX 15

/* bits (A16). Bit 1 (corner tap, D28) and bits 2..4 (inversion, rotation, D26) were removed
 * and stay 0: those follow the keyboard's devicetree. */
#define TSP_RIGHT_TAP BIT(0)
#define TSP_RIGHT_SLEEP BIT(5)
#define TSP_RIGHT_BITS_MASK (TSP_RIGHT_TAP | TSP_RIGHT_SLEEP)

enum tsp_right_field {
    TSP_RIGHT_F_SENS = 0,
    TSP_RIGHT_F_TAP = 1,
    /* 2 (corner tap) removed by D28 */
    TSP_RIGHT_F_Z = 3,
    /* 4 (separate Y threshold) removed by D27 */
    /* 5..7 (inversion, rotation) removed */
    TSP_RIGHT_F_SLEEP = 8,
};

struct tsp_right_values {
    uint8_t bits;
    uint8_t sens;
    uint8_t z;
};

struct tsp_right_state {
    bool dirty;
    bool saved;
    struct tsp_right_values cur;
    struct tsp_right_values def;
};

/* All run on the system work queue (the trackpad driver's queue) and write the sensor
 * synchronously. Return 0, -EINVAL (bad field/value) or -EIO (sensor write failed). */
void tsp_right_get(struct tsp_right_state *out);
int tsp_right_set(uint8_t field, uint8_t value);
int tsp_right_save(void);
int tsp_right_revert(void);
int tsp_right_reset(void);
