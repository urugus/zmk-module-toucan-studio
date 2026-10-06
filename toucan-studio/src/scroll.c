/*
 * Per-layer trackpad scrolling (M3, A09/A10). One node in the listener's base chain:
 *   param1 = 0: X/Y -> wheel with direction, placed before scroll-inertia
 *   param1 = 1: speed, placed after scroll-inertia and before the pointer speed processor
 * The settings of the highest active layer (by keymap order) with scrolling on apply;
 * with none, events pass through as cursor movement.
 * scroll-inertia is shared: on a layer change it gets that layer's values and is reset.
 */
#define DT_DRV_COMPAT zmk_input_processor_toucan_scroll

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>
#include <toucan_studio/scroll.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>
#include <zmk_scroll_inertia/runtime.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "exactly one toucan scroll node is supported");

#define LAYERS ZMK_KEYMAP_LAYERS_LEN
#define SETTINGS_ROOT "tsp/scroll"
#define BLOB_VERSION 1
#define BLOB_SIZE 10

#define STAGE_WHEEL 0
#define STAGE_SPEED 1

static const struct device *const inertia_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, inertia));
static const int32_t coast_num = DT_INST_PROP_BY_IDX(0, coast_ratio, 0);
static const int32_t coast_den = DT_INST_PROP_BY_IDX(0, coast_ratio, 1);

struct layer_default {
    uint8_t layer;
    struct tsp_layer_scroll v;
};

#define CHILD_DEFAULT(node)                                                                        \
    {.layer = DT_PROP(node, layer),                                                                \
     .v = {.on = 1,                                                                                \
           .speed = DT_PROP(node, speed_x10000),                                                   \
           .dir = DT_PROP(node, dir),                                                              \
           .inertia = DT_PROP(node, inertia_on),                                                   \
           .start = DT_PROP(node, start),                                                          \
           .decay = DT_PROP(node, decay)}},

static const struct layer_default child_defaults[] = {DT_INST_FOREACH_CHILD(0, CHILD_DEFAULT)};

static K_MUTEX_DEFINE(lock);
static struct tsp_layer_scroll cur[LAYERS];
static struct tsp_layer_scroll def[LAYERS];
static struct tsp_layer_scroll saved_vals[LAYERS];
static bool has_saved[LAYERS];
static int applied_layer = -2; /* layer whose values scroll-inertia has; -1 = none */

static bool id_ok(uint8_t id) { return id >= 1 && id < LAYERS; }
static bool speed_ok(uint32_t v) { return v >= TSP_SCROLL_SPEED_MIN && v <= TSP_SCROLL_SPEED_MAX; }
static bool dir_ok(uint32_t v) { return (v & ~TSP_SCROLL_DIR_MASK) == 0; }
static bool start_ok(uint32_t v) { return v >= TSP_SCROLL_START_MIN && v <= TSP_SCROLL_START_MAX; }
static bool decay_ok(uint32_t v) { return v >= TSP_SCROLL_DECAY_MIN && v <= TSP_SCROLL_DECAY_MAX; }

static bool values_ok(const struct tsp_layer_scroll *v) {
    return v->on <= 1 && speed_ok(v->speed) && dir_ok(v->dir) && v->inertia <= 1 &&
           start_ok(v->start) && decay_ok(v->decay);
}

static bool values_equal(const struct tsp_layer_scroll *a, const struct tsp_layer_scroll *b) {
    return a->on == b->on && a->speed == b->speed && a->dir == b->dir && a->inertia == b->inertia &&
           a->start == b->start && a->decay == b->decay;
}

/* Highest active layer (by keymap order) with scrolling on, or -1. */
static int active_scroll_layer(void) {
    for (int idx = LAYERS - 1; idx >= 0; idx--) {
        zmk_keymap_layer_id_t id = zmk_keymap_layer_index_to_id(idx);
        if (id == ZMK_KEYMAP_LAYER_ID_INVAL || !id_ok(id)) {
            continue;
        }
        if (cur[id].on && zmk_keymap_layer_active(id)) {
            return id;
        }
    }
    return -1;
}

/* Gives scroll-inertia the active layer's values; resets it when the layer changed.
 * Caller holds `lock`. */
static void sync_inertia(void) {
    int layer = active_scroll_layer();
    const struct tsp_layer_scroll *v = layer >= 0 ? &cur[layer] : NULL;
    struct scroll_inertia_runtime rt = {
        .enabled = v != NULL && v->inertia,
        .start = v ? v->start : TSP_SCROLL_START_MAX,
        .decay = v ? v->decay : TSP_SCROLL_DECAY_MIN,
        .scale = (v ? v->speed : TSP_SCROLL_SPEED_MIN) * coast_num,
        .scale_div = 10000 * coast_den,
    };
    int err = scroll_inertia_apply_runtime(inertia_dev, &rt);
    if (err) {
        LOG_ERR("scroll: inertia update rejected (%d)", err);
    }
    if (layer != applied_layer) {
        scroll_inertia_reset(inertia_dev);
        applied_layer = layer;
    }
}

/* ---- input processing ---------------------------------------------------------------- */

/* Only touched from the input thread. */
static int32_t rem_x, rem_y;
static int rem_layer = -1;

static int handle_event(const struct device *dev, struct input_event *event, uint32_t stage,
                        uint32_t param2, struct zmk_input_processor_state *state) {
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    if (stage == STAGE_WHEEL) {
        if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
            return ZMK_INPUT_PROC_CONTINUE;
        }
        int layer = active_scroll_layer();
        if (layer < 0) {
            return ZMK_INPUT_PROC_CONTINUE; /* cursor movement */
        }
        bool vertical = event->code == INPUT_REL_Y;
        event->code = vertical ? INPUT_REL_WHEEL : INPUT_REL_HWHEEL;
        if (cur[layer].dir & (vertical ? BIT(0) : BIT(1))) {
            event->value = -event->value;
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }
    if (event->code != INPUT_REL_WHEEL && event->code != INPUT_REL_HWHEEL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    int layer = active_scroll_layer();
    if (layer < 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    if (layer != rem_layer) {
        rem_x = rem_y = 0;
        rem_layer = layer;
    }
    int32_t *rem = event->code == INPUT_REL_WHEEL ? &rem_y : &rem_x;
    int32_t v = (int32_t)event->value * cur[layer].speed + *rem;
    int32_t scaled = v / 10000;
    *rem = v - scaled * 10000;
    event->value = CLAMP(scaled, INT16_MIN, INT16_MAX);
    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api api = {.handle_event = handle_event};

static int scroll_init(const struct device *dev) {
    const struct tsp_layer_scroll off = {
        .on = 0,
        .speed = DT_INST_PROP(0, off_speed_x10000),
        .dir = DT_INST_PROP(0, off_dir),
        .inertia = 0,
        .start = 20,
        .decay = 990,
    };
    k_mutex_lock(&lock, K_FOREVER);
    for (int i = 0; i < LAYERS; i++) {
        def[i] = off;
    }
    for (size_t i = 0; i < ARRAY_SIZE(child_defaults); i++) {
        const struct layer_default *d = &child_defaults[i];
        if (!id_ok(d->layer) || !values_ok(&d->v)) {
            LOG_ERR("scroll: invalid defaults for layer %u", d->layer);
            continue;
        }
        def[d->layer] = d->v;
    }
    memcpy(cur, def, sizeof(cur));
    sync_inertia();
    k_mutex_unlock(&lock);
    return 0;
}

DEVICE_DT_INST_DEFINE(0, scroll_init, NULL, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                      &api);

static int on_layer_state_changed(const zmk_event_t *eh) {
    k_mutex_lock(&lock, K_FOREVER);
    sync_inertia();
    k_mutex_unlock(&lock);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(toucan_scroll_layers, on_layer_state_changed);
ZMK_SUBSCRIPTION(toucan_scroll_layers, zmk_layer_state_changed);

/* ---- settings API (A09) --------------------------------------------------------------- */

uint8_t tsp_scroll_layer_count(void) { return LAYERS; }

int tsp_layer_scroll_get(uint8_t id, struct tsp_layer_scroll_state *out) {
    if (!id_ok(id)) {
        return -EINVAL;
    }
    k_mutex_lock(&lock, K_FOREVER);
    out->cur = cur[id];
    out->def = def[id];
    out->saved = has_saved[id];
    out->dirty = !values_equal(&cur[id], has_saved[id] ? &saved_vals[id] : &def[id]);
    k_mutex_unlock(&lock);
    return 0;
}

int tsp_layer_scroll_set(uint8_t id, uint8_t field, uint16_t value) {
    if (!id_ok(id)) {
        return -EINVAL;
    }
    bool ok;
    switch (field) {
    case TSP_FIELD_ON:
    case TSP_FIELD_INERTIA:
        ok = value <= 1;
        break;
    case TSP_FIELD_SPEED:
        ok = speed_ok(value);
        break;
    case TSP_FIELD_DIR:
        ok = dir_ok(value);
        break;
    case TSP_FIELD_START:
        ok = start_ok(value);
        break;
    case TSP_FIELD_DECAY:
        ok = decay_ok(value);
        break;
    default:
        ok = false;
    }
    if (!ok) {
        return -EINVAL;
    }
    k_mutex_lock(&lock, K_FOREVER);
    struct tsp_layer_scroll *v = &cur[id];
    switch (field) {
    case TSP_FIELD_ON:
        v->on = (uint8_t)value;
        break;
    case TSP_FIELD_SPEED:
        v->speed = value;
        break;
    case TSP_FIELD_DIR:
        v->dir = (uint8_t)value;
        break;
    case TSP_FIELD_INERTIA:
        v->inertia = (uint8_t)value;
        break;
    case TSP_FIELD_START:
        v->start = value;
        break;
    case TSP_FIELD_DECAY:
        v->decay = value;
        break;
    }
    sync_inertia();
    k_mutex_unlock(&lock);
    return 0;
}

static void settings_key(uint8_t id, char *buf, size_t len) { snprintf(buf, len, SETTINGS_ROOT "/%u", id); }

int tsp_layer_scroll_save(uint8_t id) {
    if (!id_ok(id)) {
        return -EINVAL;
    }
    char key[24];
    uint8_t b[BLOB_SIZE];
    settings_key(id, key, sizeof(key));
    k_mutex_lock(&lock, K_FOREVER);
    const struct tsp_layer_scroll *v = &cur[id];
    b[0] = BLOB_VERSION;
    b[1] = v->on;
    sys_put_le16(v->speed, b + 2);
    b[4] = v->dir;
    b[5] = v->inertia;
    sys_put_le16(v->start, b + 6);
    sys_put_le16(v->decay, b + 8);
    int err = settings_save_one(key, b, sizeof(b));
    if (err == 0) {
        saved_vals[id] = *v;
        has_saved[id] = true;
    } else {
        LOG_ERR("scroll: save of layer %u failed (%d)", id, err);
    }
    k_mutex_unlock(&lock);
    return err;
}

int tsp_layer_scroll_revert(uint8_t id) {
    if (!id_ok(id)) {
        return -EINVAL;
    }
    k_mutex_lock(&lock, K_FOREVER);
    cur[id] = has_saved[id] ? saved_vals[id] : def[id];
    sync_inertia();
    k_mutex_unlock(&lock);
    return 0;
}

int tsp_layer_scroll_reset(uint8_t id) {
    if (!id_ok(id)) {
        return -EINVAL;
    }
    char key[24];
    settings_key(id, key, sizeof(key));
    k_mutex_lock(&lock, K_FOREVER);
    int err = settings_delete(key);
    if (err == 0) {
        has_saved[id] = false;
        cur[id] = def[id];
        sync_inertia();
    } else {
        LOG_ERR("scroll: delete of layer %u failed (%d)", id, err);
    }
    k_mutex_unlock(&lock);
    return err;
}

/* The two-mode trial build stored everything under "tsp/scroll" (A07). Remove it after
 * loading; deleting from inside the load callback would re-enter the settings subsystem. */
static void delete_legacy(struct k_work *work) {
    int err = settings_delete(SETTINGS_ROOT);
    LOG_INF("scroll: removed legacy settings (%d)", err);
}

static K_WORK_DEFINE(delete_legacy_work, delete_legacy);

static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    if (name == NULL || name[0] == '\0') {
        k_work_submit(&delete_legacy_work);
        return 0;
    }
    char *end;
    unsigned long id = strtoul(name, &end, 10);
    if (*end != '\0' || id > UINT8_MAX || !id_ok((uint8_t)id)) {
        return -ENOENT;
    }
    uint8_t b[BLOB_SIZE];
    if (len != sizeof(b) || read_cb(cb_arg, b, sizeof(b)) != sizeof(b) || b[0] != BLOB_VERSION) {
        LOG_WRN("scroll: ignoring stored settings for layer %lu", id);
        return 0;
    }
    struct tsp_layer_scroll v = {
        .on = b[1],
        .speed = sys_get_le16(b + 2),
        .dir = b[4],
        .inertia = b[5],
        .start = sys_get_le16(b + 6),
        .decay = sys_get_le16(b + 8),
    };
    if (!values_ok(&v)) {
        LOG_WRN("scroll: ignoring out-of-range settings for layer %lu", id);
        return 0;
    }
    k_mutex_lock(&lock, K_FOREVER);
    saved_vals[id] = v;
    has_saved[id] = true;
    cur[id] = v;
    sync_inertia();
    k_mutex_unlock(&lock);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(tsp_scroll, SETTINGS_ROOT, NULL, settings_set, NULL, NULL);
