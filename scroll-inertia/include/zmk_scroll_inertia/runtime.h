/*
 * Toucan Studio addition (not upstream): change a few parameters at runtime.
 * Values use the same units as the devicetree properties.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

struct scroll_inertia_runtime {
    /* false: never start inertia; active scrolling still passes through. */
    bool enabled;
    /* `start` property (scroll units). */
    int32_t start;
    /* Applied to decay-fast, decay-slow and decay-tail (permille per tick). */
    int32_t decay;
    /* `scale` / `scale-div` properties; both must be > 0. */
    int32_t scale;
    int32_t scale_div;
};

/* Returns -EINVAL for out-of-range values (nothing is changed then). */
int scroll_inertia_apply_runtime(const struct device *dev, const struct scroll_inertia_runtime *rt);

/* Drops any tracking/coasting state (e.g. when the scroll settings switch layers). */
void scroll_inertia_reset(const struct device *dev);
