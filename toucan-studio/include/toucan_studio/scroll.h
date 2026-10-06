#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Per-layer scroll settings (A09/A10, docs/DECISIONS.md). Layers are addressed by id. */
#define TSP_SCROLL_SPEED_MIN 100 /* x10000 */
#define TSP_SCROLL_SPEED_MAX 10000
#define TSP_SCROLL_DIR_MASK 0x03 /* bit0 invert vertical, bit1 invert horizontal */
#define TSP_SCROLL_START_MIN 5
#define TSP_SCROLL_START_MAX 200
#define TSP_SCROLL_DECAY_MIN 900
#define TSP_SCROLL_DECAY_MAX 998

enum tsp_scroll_field {
    TSP_FIELD_ON = 0,
    TSP_FIELD_SPEED = 1,
    TSP_FIELD_DIR = 2,
    TSP_FIELD_INERTIA = 3,
    TSP_FIELD_START = 4,
    TSP_FIELD_DECAY = 5,
};

struct tsp_layer_scroll {
    uint8_t on;
    uint16_t speed;
    uint8_t dir;
    uint8_t inertia;
    uint16_t start;
    uint16_t decay;
};

struct tsp_layer_scroll_state {
    /* Current values differ from what applies after a restart (saved, else default). */
    bool dirty;
    bool saved;
    struct tsp_layer_scroll cur;
    struct tsp_layer_scroll def;
};

/* Number of layer ids; settings exist for ids 1..count-1. */
uint8_t tsp_scroll_layer_count(void);
/* All return -EINVAL for id 0 or an unknown id. */
int tsp_layer_scroll_get(uint8_t id, struct tsp_layer_scroll_state *out);
/* RAM only; -EINVAL also for an unknown field or out-of-range value. */
int tsp_layer_scroll_set(uint8_t id, uint8_t field, uint16_t value);
int tsp_layer_scroll_save(uint8_t id);
int tsp_layer_scroll_revert(uint8_t id);
int tsp_layer_scroll_reset(uint8_t id);
