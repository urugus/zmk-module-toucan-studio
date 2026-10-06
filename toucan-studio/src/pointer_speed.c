/*
 * Runtime-adjustable pointer speed (M2). Replaces `&zip_xy_scaler 9 4` on the left half.
 *
 * Differences from ZMK's scaler: one remainder per axis (the stock scaler shares one
 * between X and Y) and 32-bit intermediate values, since value * 800 overflows int16.
 */
#define DT_DRV_COMPAT zmk_input_processor_toucan_pointer_speed

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>
#include <toucan_studio/pointer_speed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
             "exactly one toucan pointer speed processor is supported");

#define DEFAULT_SCALE DT_INST_PROP(0, default_scale_x100)
BUILD_ASSERT(DEFAULT_SCALE >= TSP_PTR_SPEED_MIN && DEFAULT_SCALE <= TSP_PTR_SPEED_MAX,
             "default-scale-x100 out of range");

#define SETTINGS_KEY "tsp/ptr/speed"

static atomic_t cur = ATOMIC_INIT(DEFAULT_SCALE);
static uint16_t saved; /* 0 = nothing saved */
static K_MUTEX_DEFINE(lock);

/* Only touched from the input thread. */
static int32_t remainder_x;
static int32_t remainder_y;

static bool in_range(uint32_t v) { return v >= TSP_PTR_SPEED_MIN && v <= TSP_PTR_SPEED_MAX; }

static int scale(struct input_event *event, int32_t *remainder) {
    int32_t v = (int32_t)event->value * (int32_t)atomic_get(&cur) + *remainder;
    int32_t scaled = v / 100;
    *remainder = v - scaled * 100;
    event->value = CLAMP(scaled, INT16_MIN, INT16_MAX);
    return ZMK_INPUT_PROC_CONTINUE;
}

static int handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                        uint32_t param2, struct zmk_input_processor_state *state) {
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    switch (event->code) {
    case INPUT_REL_X:
        return scale(event, &remainder_x);
    case INPUT_REL_Y:
        return scale(event, &remainder_y);
    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }
}

static const struct zmk_input_processor_driver_api api = {.handle_event = handle_event};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                      &api);

void tsp_ptr_speed_get(struct tsp_ptr_speed_state *out) {
    k_mutex_lock(&lock, K_FOREVER);
    *out = (struct tsp_ptr_speed_state){
        .cur = (uint16_t)atomic_get(&cur),
        .def = DEFAULT_SCALE,
        .min = TSP_PTR_SPEED_MIN,
        .max = TSP_PTR_SPEED_MAX,
        .saved = saved,
    };
    k_mutex_unlock(&lock);
}

int tsp_ptr_speed_set(uint16_t value) {
    if (!in_range(value)) {
        return -EINVAL;
    }
    atomic_set(&cur, value);
    return 0;
}

int tsp_ptr_speed_save(void) {
    k_mutex_lock(&lock, K_FOREVER);
    uint16_t value = (uint16_t)atomic_get(&cur);
    int err = settings_save_one(SETTINGS_KEY, &value, sizeof(value));
    if (err == 0) {
        saved = value;
    } else {
        LOG_ERR("pointer speed: save failed (%d)", err);
    }
    k_mutex_unlock(&lock);
    return err;
}

void tsp_ptr_speed_revert(void) {
    k_mutex_lock(&lock, K_FOREVER);
    atomic_set(&cur, saved ? saved : DEFAULT_SCALE);
    k_mutex_unlock(&lock);
}

int tsp_ptr_speed_reset(void) {
    k_mutex_lock(&lock, K_FOREVER);
    int err = settings_delete(SETTINGS_KEY);
    if (err == 0) {
        saved = 0;
        atomic_set(&cur, DEFAULT_SCALE);
    } else {
        LOG_ERR("pointer speed: delete failed (%d)", err);
    }
    k_mutex_unlock(&lock);
    return err;
}

static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    if (strcmp(name, "ptr/speed") != 0) {
        return -ENOENT;
    }
    uint16_t value;
    if (len != sizeof(value) || read_cb(cb_arg, &value, sizeof(value)) != sizeof(value)) {
        return -EINVAL;
    }
    /* Ignore corrupt or out-of-range values and keep the default (A05). */
    if (!in_range(value)) {
        LOG_WRN("pointer speed: ignoring stored value %u", value);
        return 0;
    }
    k_mutex_lock(&lock, K_FOREVER);
    saved = value;
    atomic_set(&cur, value);
    k_mutex_unlock(&lock);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(tsp, "tsp", NULL, settings_set, NULL, NULL);
