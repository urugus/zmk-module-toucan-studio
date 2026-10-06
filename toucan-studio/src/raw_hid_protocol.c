/*
 * Toucan Studio settings protocol over raw HID (A03, docs/DECISIONS.md).
 *
 * A09 adds the per-layer scroll commands (0x20-0x24); A18 adds get_build (0x02) and
 * enter_bootloader (0x03) for app-driven USB updates (M7); A24 adds enter_ble_dfu (0x04) so the
 * Mac can write the left half over Bluetooth like the right half (M10); A27 adds get_storage
 * (0x05), the free space of the settings storage shared by the keymap, pairings and settings;
 * A30 adds unlock (0x06), the same call as the &studio_unlock key, so the app can unlock
 * Studio right after connecting.
 * A28 (M14) makes the three requests that reboot a half (0x03, 0x04 and the relayed 0x36) wait
 * for a key press on either half, so a program on the Mac cannot rewrite the keyboard unseen.
 *
 * request:  [ver=1, seq, cmd, payload...]
 * response: [ver=1, seq, status, payload...]   little-endian, zero padded
 *
 * raw_hid_received_event arrives from the USB/BLE callback context with a
 * short-lived data pointer, so reports are copied into a queue and handled in
 * order on the system work queue. zmk-raw-hid sends each reply over both USB
 * and BLE; the host matches replies by seq and ignores the rest.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/fs/nvs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>

#include <raw_hid/events.h>
#include <toucan_studio/build_info.h>
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_POINTER_SPEED)
#include <toucan_studio/pointer_speed.h>
#endif
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_SCROLL)
#include <toucan_studio/scroll.h>
#endif
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_RIGHT_RELAY)
#include <toucan_studio/right_link.h>
#endif
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#if IS_ENABLED(CONFIG_ZMK_STUDIO)
#include <zmk/studio/core.h>
#endif
#include <zmk/usb.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PROTO_VERSION 1
#define MODULE_VERSION 5
#define FEATURE_POINTER_SPEED BIT(0)
#define FEATURE_SCROLL BIT(1) /* per-layer scroll (A09) */
#define FEATURE_RIGHT BIT(2)  /* right-half trackpad settings (A14) */
#define FEATURE_UPDATE BIT(3) /* get_build / enter_bootloader (A18) */
#define FEATURE_RIGHT_UPDATE BIT(4) /* get_right_build / right_enter_dfu relayed (A21) */
#define FEATURE_BLE_UPDATE BIT(5)   /* enter_ble_dfu (A24) */
#define FEATURE_STORAGE BIT(6)      /* get_storage (A27) */
#define FEATURE_CONFIRM BIT(7)      /* reboots wait for a key press (A28) */
#define FEATURE_UNLOCK BIT(8)       /* unlock (A30) */
#define FEATURES                                                                                   \
    ((IS_ENABLED(CONFIG_TOUCAN_STUDIO_POINTER_SPEED) ? FEATURE_POINTER_SPEED : 0) |               \
     (IS_ENABLED(CONFIG_TOUCAN_STUDIO_SCROLL) ? FEATURE_SCROLL : 0) |                              \
     (IS_ENABLED(CONFIG_TOUCAN_STUDIO_RIGHT_RELAY) ? FEATURE_RIGHT | FEATURE_RIGHT_UPDATE : 0) | FEATURE_UPDATE | FEATURE_BLE_UPDATE |   \
     (IS_ENABLED(CONFIG_SETTINGS_NVS) ? FEATURE_STORAGE : 0) | FEATURE_CONFIRM |                  \
     (IS_ENABLED(CONFIG_ZMK_STUDIO) ? FEATURE_UNLOCK : 0))

/* GPREGRET value the Adafruit bootloader reads as "stay in UF2 drive mode" (ZMK RST_UF2). */
#define BOOTLOADER_UF2 0x57
/* ... and "start BLE OTA DFU" (DFU_MAGIC_OTA_RESET). */
#define BOOTLOADER_OTA 0xA8
#define BOOTLOADER_DELAY K_MSEC(200)
/* How long a reboot request waits for a key press (A28). The host allows 12 s. */
#define CONFIRM_TIMEOUT K_SECONDS(10)
static const uint8_t bootloader_key[] = {'B', 'O', 'O', 'T'};

enum cmd {
    CMD_GET_INFO = 0x01,
    CMD_GET_BUILD = 0x02,
    CMD_ENTER_BOOTLOADER = 0x03,
    CMD_ENTER_BLE_DFU = 0x04,
    CMD_GET_STORAGE = 0x05,
    CMD_UNLOCK = 0x06,
    CMD_GET_PTR_SPEED = 0x10,
    CMD_SET_PTR_SPEED = 0x11,
    CMD_SAVE_PTR_SPEED = 0x12,
    CMD_REVERT_PTR_SPEED = 0x13,
    CMD_RESET_PTR_SPEED = 0x14,
    CMD_GET_LAYER_SCROLL = 0x20,
    CMD_SET_LAYER_SCROLL = 0x21,
    CMD_SAVE_LAYER_SCROLL = 0x22,
    CMD_REVERT_LAYER_SCROLL = 0x23,
    CMD_RESET_LAYER_SCROLL = 0x24,
};

enum status {
    STATUS_OK = 0,
    STATUS_UNKNOWN_CMD = 1,
    STATUS_OUT_OF_RANGE = 2,
    STATUS_STORAGE = 3,
    STATUS_BAD_VERSION = 4,
    /* 5-7 are the right-half statuses (right_link.h). */
    STATUS_NO_USB = 8,
    STATUS_NOT_CONFIRMED = 9, /* no key was pressed within CONFIRM_TIMEOUT (A28) */
};

#define REPORT_SIZE CONFIG_RAW_HID_REPORT_SIZE
BUILD_ASSERT(REPORT_SIZE >= 3 + 20, "reports must hold the layer scroll state");
BUILD_ASSERT(REPORT_SIZE >= 3 + 1 + TSP_BUILD_ID_LEN + TSP_BUILD_LABEL_LEN, "reports must hold get_build");

struct report {
    uint8_t bytes[REPORT_SIZE];
};

/* The host keeps one request in flight; a little depth absorbs retries. */
K_MSGQ_DEFINE(requests, sizeof(struct report), 4, 4);

static struct report reply; /* only used from the work item */

#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_POINTER_SPEED)
static size_t put_ptr_state(uint8_t *p) {
    struct tsp_ptr_speed_state s;
    tsp_ptr_speed_get(&s);
    sys_put_le16(s.cur, p);
    sys_put_le16(s.def, p + 2);
    sys_put_le16(s.min, p + 4);
    sys_put_le16(s.max, p + 6);
    sys_put_le16(s.saved, p + 8);
    return 10;
}
#endif

#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_SCROLL)
static size_t put_layer_values(const struct tsp_layer_scroll *v, uint8_t *p) {
    p[0] = v->on;
    sys_put_le16(v->speed, p + 1);
    p[3] = v->dir;
    p[4] = v->inertia;
    sys_put_le16(v->start, p + 5);
    sys_put_le16(v->decay, p + 7);
    return 9;
}

/* [id, flags(bit0 dirty, bit1 saved), cur(9), def(9)] = 20 bytes (A09). */
static size_t put_layer_state(uint8_t id, const struct tsp_layer_scroll_state *s, uint8_t *p) {
    p[0] = id;
    p[1] = (s->dirty ? BIT(0) : 0) | (s->saved ? BIT(1) : 0);
    size_t n = 2;
    n += put_layer_values(&s->cur, p + n);
    n += put_layer_values(&s->def, p + n);
    return n;
}
#endif

#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_POINTER_SPEED)
static uint8_t handle_ptr(uint8_t cmd, const uint8_t *arg, uint8_t *out, size_t *out_len) {
    uint8_t status = STATUS_OK;
    switch (cmd) {
    case CMD_GET_PTR_SPEED:
        break;
    case CMD_SET_PTR_SPEED:
        status = tsp_ptr_speed_set(sys_get_le16(arg)) ? STATUS_OUT_OF_RANGE : STATUS_OK;
        break;
    case CMD_SAVE_PTR_SPEED:
        status = tsp_ptr_speed_save() ? STATUS_STORAGE : STATUS_OK;
        break;
    case CMD_REVERT_PTR_SPEED:
        tsp_ptr_speed_revert();
        break;
    case CMD_RESET_PTR_SPEED:
        status = tsp_ptr_speed_reset() ? STATUS_STORAGE : STATUS_OK;
        break;
    default:
        *out_len = 0;
        return STATUS_UNKNOWN_CMD;
    }
    *out_len = put_ptr_state(out);
    return status;
}
#endif

#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_SCROLL)
/* Every command starts with the layer id: [id, ...]. */
static uint8_t handle_scroll(uint8_t cmd, const uint8_t *arg, uint8_t *out, size_t *out_len) {
    uint8_t id = arg[0];
    int err = 0;
    uint8_t fail = STATUS_OUT_OF_RANGE;
    switch (cmd) {
    case CMD_GET_LAYER_SCROLL:
        break;
    case CMD_SET_LAYER_SCROLL: /* [id, field u8, value u16] */
        err = tsp_layer_scroll_set(id, arg[1], sys_get_le16(arg + 2));
        break;
    case CMD_SAVE_LAYER_SCROLL:
        err = tsp_layer_scroll_save(id);
        fail = err == -EINVAL ? STATUS_OUT_OF_RANGE : STATUS_STORAGE;
        break;
    case CMD_REVERT_LAYER_SCROLL:
        err = tsp_layer_scroll_revert(id);
        break;
    case CMD_RESET_LAYER_SCROLL:
        err = tsp_layer_scroll_reset(id);
        fail = err == -EINVAL ? STATUS_OUT_OF_RANGE : STATUS_STORAGE;
        break;
    default:
        return STATUS_UNKNOWN_CMD;
    }
    struct tsp_layer_scroll_state s;
    if (tsp_layer_scroll_get(id, &s) != 0) {
        return STATUS_OUT_OF_RANGE; /* unknown layer: no state to report */
    }
    *out_len = put_layer_state(id, &s, out);
    return err ? fail : STATUS_OK;
}
#endif

/* Runs after the reply to a confirmed reboot request has had time to reach the host. */
static uint8_t bootloader_mode = BOOTLOADER_UF2;

static void reboot_to_bootloader(struct k_work *work) { sys_reboot(bootloader_mode); }

static K_WORK_DELAYABLE_DEFINE(bootloader_work, reboot_to_bootloader);

/* [side, id(8), label(20)] (A18). */
static size_t put_build(uint8_t *p) {
    p[0] = tsp_build_info.side;
    memcpy(p + 1, tsp_build_info.id, TSP_BUILD_ID_LEN);
    memcpy(p + 1 + TSP_BUILD_ID_LEN, tsp_build_info.label, TSP_BUILD_LABEL_LEN);
    return 1 + TSP_BUILD_ID_LEN + TSP_BUILD_LABEL_LEN;
}

#if IS_ENABLED(CONFIG_SETTINGS_NVS)
/* A27: [free u16, total u16] in bytes. One sector is kept for garbage collection, so the
 * usable size is the other sectors; `free` is what remains after stale records are collected. */
static uint8_t get_storage(uint8_t *out, size_t *out_len) {
    void *storage;
    if (settings_storage_get(&storage) != 0) {
        return STATUS_STORAGE;
    }
    struct nvs_fs *fs = storage;
    ssize_t free = nvs_calc_free_space(fs);
    if (free < 0) {
        return STATUS_STORAGE;
    }
    sys_put_le16(MIN(free, UINT16_MAX), out);
    sys_put_le16(MIN((size_t)fs->sector_size * (fs->sector_count - 1), UINT16_MAX), out + 2);
    *out_len = 4;
    return STATUS_OK;
}
#endif

/* Fills `out` (payload area after the 3-byte header) and returns the status. */
static uint8_t handle(uint8_t cmd, const uint8_t *arg, uint8_t *out, size_t *out_len) {
    *out_len = 0;
    if (cmd == CMD_GET_INFO) {
        sys_put_le16(MODULE_VERSION, out);
        sys_put_le16(FEATURES, out + 2);
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_SCROLL)
        out[4] = tsp_scroll_layer_count();
#else
        out[4] = 0;
#endif
        *out_len = 5;
        return STATUS_OK;
    }
    if (cmd == CMD_GET_BUILD) {
        *out_len = put_build(out);
        return STATUS_OK;
    }
#if IS_ENABLED(CONFIG_SETTINGS_NVS)
    if (cmd == CMD_GET_STORAGE) {
        return get_storage(out, out_len);
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_STUDIO)
    if (cmd == CMD_UNLOCK) {
        /* A30: exactly what the &studio_unlock key does. The new state reaches the host through
         * the Studio notification; ZMK re-locks on its own (idle timeout, disconnect), so
         * nothing is stored. */
        zmk_studio_core_unlock();
        return STATUS_OK;
    }
#endif
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_POINTER_SPEED)
    if (cmd >= CMD_GET_PTR_SPEED && cmd <= CMD_RESET_PTR_SPEED) {
        return handle_ptr(cmd, arg, out, out_len);
    }
#endif
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_SCROLL)
    if (cmd >= CMD_GET_LAYER_SCROLL && cmd <= CMD_RESET_LAYER_SCROLL) {
        return handle_scroll(cmd, arg, out, out_len);
    }
#endif
    return STATUS_UNKNOWN_CMD;
}

static void send_status(uint8_t seq, uint8_t status) {
    memset(reply.bytes, 0, sizeof(reply.bytes));
    reply.bytes[0] = PROTO_VERSION;
    reply.bytes[1] = seq;
    reply.bytes[2] = status;
    raise_raw_hid_sent_event((struct raw_hid_sent_event){.data = reply.bytes, .length = 3});
}

#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_RIGHT_RELAY)
/* Right-half requests (A14) are relayed asynchronously; the reply goes out when the right
 * half answers or after RIGHT_TIMEOUT. Everything below runs on the system work queue. */
#define RIGHT_TIMEOUT K_SECONDS(1)

static uint8_t right_seq;
static uint8_t right_reply[TSP_RIGHT_REPLY_MAX];
static size_t right_reply_len;

static void right_timeout(struct k_work *work) {
    tsp_right_relay_cancel();
    send_status(right_seq, TSP_STATUS_RIGHT_NO_REPLY);
}

static K_WORK_DELAYABLE_DEFINE(right_timeout_work, right_timeout);

static void right_done(struct k_work *work) {
    k_work_cancel_delayable(&right_timeout_work);
    if (right_reply_len == 0) {
        send_status(right_seq, tsp_right_relay_ready() ? TSP_STATUS_RIGHT_NO_REPLY : TSP_STATUS_RIGHT_NOT_CONNECTED);
        return;
    }
    memset(reply.bytes, 0, sizeof(reply.bytes));
    memcpy(reply.bytes, right_reply, right_reply_len);
    raise_raw_hid_sent_event((struct raw_hid_sent_event){.data = reply.bytes, .length = right_reply_len});
}

static K_WORK_DEFINE(right_done_work, right_done);

/* Called from the Bluetooth stack's context: copy and hand over to the work queue. */
static void on_right_reply(const uint8_t *data, size_t len) {
    right_reply_len = data ? MIN(len, sizeof(right_reply)) : 0;
    if (data) {
        memcpy(right_reply, data, right_reply_len);
    }
    k_work_submit(&right_done_work);
}

static void start_right(const struct report *req) {
    right_seq = req->bytes[1];
    size_t len = req->bytes[2] == TSP_CMD_RIGHT_DFU ? TSP_RIGHT_REQ_MAX : TSP_RIGHT_REQ_LEN;
    int err = tsp_right_relay_send(req->bytes, len, on_right_reply);
    if (err == -ENOTCONN) {
        /* The diagnostics payload locates where link preparation stopped. */
        memset(reply.bytes, 0, sizeof(reply.bytes));
        reply.bytes[0] = PROTO_VERSION;
        reply.bytes[1] = right_seq;
        reply.bytes[2] = TSP_STATUS_RIGHT_NOT_CONNECTED;
        tsp_right_relay_diag(&reply.bytes[3]);
        raise_raw_hid_sent_event((struct raw_hid_sent_event){.data = reply.bytes, .length = 3 + TSP_RIGHT_DIAG_LEN});
        return;
    }
    if (err) {
        send_status(right_seq, TSP_STATUS_RIGHT_NO_REPLY);
        return;
    }
    k_work_reschedule(&right_timeout_work, RIGHT_TIMEOUT);
}
#endif

/*
 * A28: a request that reboots a half is held until a key is pressed on either half (the left
 * half sees the right half's positions too). Without a key press within CONFIRM_TIMEOUT it is
 * answered with STATUS_NOT_CONFIRMED and nothing happens. Everything here runs on the system
 * work queue except on_position, which only submits work.
 */
static struct report pending; /* valid while pending_valid */
static bool pending_valid;

static bool needs_confirm(uint8_t cmd) {
    if (cmd == CMD_ENTER_BOOTLOADER || cmd == CMD_ENTER_BLE_DFU) {
        return true;
    }
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_RIGHT_RELAY)
    if (cmd == TSP_CMD_RIGHT_DFU) {
        return true;
    }
#endif
    return false;
}

/* The refusals that come back at once, as before: a wrong key, or 0x03 without a USB host
 * (the bootloader's UF2 mode returns to the application after 3 s without one, M8.1, so this
 * is only a guard against a pointless reboot). -1 means hold the request for a key press. */
static int precheck(const struct report *req) {
    uint8_t cmd = req->bytes[2];
    const uint8_t *arg = &req->bytes[3];
    if (cmd == CMD_ENTER_BOOTLOADER || cmd == CMD_ENTER_BLE_DFU) {
        if (memcmp(arg, bootloader_key, sizeof(bootloader_key)) != 0) {
            return STATUS_OUT_OF_RANGE;
        }
        if (cmd == CMD_ENTER_BOOTLOADER && zmk_usb_get_conn_state() != ZMK_USB_CONN_HID) {
            return STATUS_NO_USB;
        }
    }
    return -1;
}

static void confirm_timeout(struct k_work *work) {
    if (!pending_valid) {
        return;
    }
    pending_valid = false;
    send_status(pending.bytes[1], STATUS_NOT_CONFIRMED);
}

static K_WORK_DELAYABLE_DEFINE(confirm_timeout_work, confirm_timeout);

static void run_confirmed(struct k_work *work) {
    if (!pending_valid) {
        return;
    }
    pending_valid = false;
    k_work_cancel_delayable(&confirm_timeout_work);
    struct report req = pending;
    switch (req.bytes[2]) {
    case CMD_ENTER_BOOTLOADER:
        bootloader_mode = BOOTLOADER_UF2;
        break;
    case CMD_ENTER_BLE_DFU:
        /* A24: the bootloader advertises its DFU service and waits for the Mac without a
         * timeout; a single reset brings the keyboard back if nothing arrives. */
        bootloader_mode = BOOTLOADER_OTA;
        break;
    default:
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_RIGHT_RELAY)
        start_right(&req); /* the right half answers and reboots itself (A21) */
#endif
        return;
    }
    send_status(req.bytes[1], STATUS_OK);
    k_work_schedule(&bootloader_work, BOOTLOADER_DELAY);
}

static K_WORK_DEFINE(confirmed_work, run_confirmed);

/* A newer request replaces the one waiting, which is answered as not confirmed. */
static void hold(const struct report *req) {
    if (pending_valid) {
        send_status(pending.bytes[1], STATUS_NOT_CONFIRMED);
    }
    pending = *req;
    pending_valid = true;
    k_work_reschedule(&confirm_timeout_work, CONFIRM_TIMEOUT);
}

/* Only presses that arrive while a request waits count; a key held before it does not. */
static int on_position(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev != NULL && ev->state && pending_valid) {
        k_work_submit(&confirmed_work);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(toucan_studio_confirm, on_position);
ZMK_SUBSCRIPTION(toucan_studio_confirm, zmk_position_state_changed);

static void process(struct k_work *work) {
    struct report req;
    while (k_msgq_get(&requests, &req, K_NO_WAIT) == 0) {
        if (req.bytes[0] == PROTO_VERSION && needs_confirm(req.bytes[2])) {
            int s = precheck(&req);
            if (s < 0) {
                hold(&req);
            } else {
                send_status(req.bytes[1], s);
            }
            continue;
        }
#if IS_ENABLED(CONFIG_TOUCAN_STUDIO_RIGHT_RELAY)
        if (req.bytes[0] == PROTO_VERSION && req.bytes[2] >= TSP_CMD_RIGHT_FIRST && req.bytes[2] <= TSP_CMD_RIGHT_LAST) {
            start_right(&req);
            continue;
        }
#endif
        uint8_t *r = reply.bytes;
        size_t payload_len = 0;
        memset(r, 0, sizeof(reply.bytes));
        r[0] = PROTO_VERSION;
        r[1] = req.bytes[1];
        if (req.bytes[0] != PROTO_VERSION) {
            r[2] = STATUS_BAD_VERSION;
        } else {
            r[2] = handle(req.bytes[2], &req.bytes[3], &r[3], &payload_len);
        }
        raise_raw_hid_sent_event((struct raw_hid_sent_event){.data = r, .length = 3 + payload_len});
    }
}

static K_WORK_DEFINE(process_work, process);

static int on_raw_hid_received(const zmk_event_t *eh) {
    const struct raw_hid_received_event *ev = as_raw_hid_received_event(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    struct report req = {0};
    memcpy(req.bytes, ev->data, MIN(ev->length, REPORT_SIZE));
    if (k_msgq_put(&requests, &req, K_NO_WAIT) != 0) {
        /* Only when the host ignores A03's one-in-flight rule; it will time out. */
        LOG_WRN("toucan studio: request queue full, dropping seq %u", req.bytes[1]);
    }
    k_work_submit(&process_work);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(toucan_studio_raw_hid, on_raw_hid_received);
ZMK_SUBSCRIPTION(toucan_studio_raw_hid, raw_hid_received_event);
