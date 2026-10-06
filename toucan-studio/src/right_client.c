/*
 * Right half (peripheral): answers the left half's settings requests (M6, A14/A15).
 * The link service lives on the left half (D25): once the split link is encrypted, this side
 * discovers it and subscribes. Requests arrive as notifications, are handled on the system
 * work queue and answered with a write without response.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>

#include <toucan_studio/build_info.h>
#include <toucan_studio/right_link.h>
#include <toucan_studio/right_trackpad.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PROTO_VERSION 1

enum {
    CMD_GET = 0x30,
    CMD_SET = 0x31,
    CMD_SAVE = 0x32,
    CMD_REVERT = 0x33,
    CMD_RESET = 0x34,
};

static const struct bt_uuid_128 svc_uuid = BT_UUID_INIT_128(TSP_RIGHT_SERVICE_UUID);
static const struct bt_uuid_128 chrc_uuid = BT_UUID_INIT_128(TSP_RIGHT_CHRC_UUID);
/* Not BT_UUID_GATT_CCC: that is a compound literal, which would not outlive on_discover()
 * while the stack keeps the pointer for the next discovery step. */
static const struct bt_uuid_16 ccc_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

static uint8_t req[TSP_RIGHT_REQ_MAX];
static uint8_t reply[TSP_RIGHT_REPLY_MAX];
static size_t reply_len;
static atomic_t busy; /* one request at a time (the left half keeps one in flight) */

static struct bt_conn *conn;  /* the split link, while known */
static uint16_t value_handle; /* 0 until subscribed */
static struct bt_gatt_discover_params disc;
static struct bt_gatt_subscribe_params sub;

static void put_values(const struct tsp_right_values *v, uint8_t *p) {
    p[0] = v->bits;
    p[1] = v->sens;
    p[2] = v->z;
}

static uint8_t status_of(int err) {
    switch (err) {
    case 0:
        return TSP_STATUS_OK;
    case -EINVAL:
        return TSP_STATUS_OUT_OF_RANGE;
    case -EIO:
        return TSP_STATUS_SENSOR;
    default:
        return TSP_STATUS_STORAGE;
    }
}

/* GPREGRET value the Adafruit bootloader reads as "start BLE OTA DFU" (DFU_MAGIC_OTA_RESET). */
#define BOOTLOADER_OTA 0xA8
#define BOOTLOADER_DELAY K_MSEC(200)
static const uint8_t dfu_key[] = {'B', 'O', 'O', 'T'};

/* Runs after the reply has had time to reach the left half. */
static void reboot_to_dfu(struct k_work *work) { sys_reboot(BOOTLOADER_OTA); }

static K_WORK_DELAYABLE_DEFINE(dfu_work, reboot_to_dfu);

static void build_reply(void) {
    memset(reply, 0, sizeof(reply));
    reply_len = TSP_RIGHT_REPLY_LEN;
    reply[0] = PROTO_VERSION;
    reply[1] = req[1];
    if (req[0] != PROTO_VERSION) {
        reply[2] = TSP_STATUS_BAD_VERSION;
        return;
    }
    if (req[2] == TSP_CMD_RIGHT_BUILD) { /* [side, id(8)]; the label stays home (A21) */
        reply[2] = TSP_STATUS_OK;
        reply[3] = tsp_build_info.side;
        memcpy(&reply[4], tsp_build_info.id, TSP_BUILD_ID_LEN);
        reply_len = TSP_RIGHT_REPLY_MAX;
        return;
    }
    if (req[2] == TSP_CMD_RIGHT_DFU) {
        /* The bootloader waits for the Mac without a timeout; a single reset brings the
         * keyboard back if nothing arrives. */
        bool ok = memcmp(&req[3], dfu_key, sizeof(dfu_key)) == 0;
        reply[2] = ok ? TSP_STATUS_OK : TSP_STATUS_OUT_OF_RANGE;
        if (ok) {
            k_work_schedule(&dfu_work, BOOTLOADER_DELAY);
        }
        return;
    }
    int err = 0;
    switch (req[2]) {
    case CMD_GET:
        break;
    case CMD_SET:
        err = tsp_right_set(req[3], req[4]);
        break;
    case CMD_SAVE:
        err = tsp_right_save();
        break;
    case CMD_REVERT:
        err = tsp_right_revert();
        break;
    case CMD_RESET:
        err = tsp_right_reset();
        break;
    default:
        reply[2] = TSP_STATUS_UNKNOWN_CMD;
        return;
    }
    struct tsp_right_state s;
    tsp_right_get(&s);
    reply[2] = status_of(err);
    reply[3] = (s.dirty ? BIT(0) : 0) | (s.saved ? BIT(1) : 0);
    put_values(&s.cur, &reply[4]);
    put_values(&s.def, &reply[7]);
}

static void handle(struct k_work *work) {
    build_reply();
    if (conn == NULL || value_handle == 0 ||
        bt_gatt_write_without_response(conn, value_handle, reply, reply_len, false) != 0) {
        LOG_WRN("right link: cannot reply"); /* the left half times out and reports it */
    }
    atomic_clear(&busy);
}

static K_WORK_DEFINE(handle_work, handle);

static uint8_t on_notify(struct bt_conn *c, struct bt_gatt_subscribe_params *params, const void *data,
                         uint16_t len) {
    if (data == NULL) {
        value_handle = 0; /* unsubscribed */
        return BT_GATT_ITER_STOP;
    }
    LOG_INF("right link: request (%u bytes)", len);
    if (len < 3 || len > sizeof(req) || !atomic_cas(&busy, 0, 1)) {
        return BT_GATT_ITER_CONTINUE; /* the left half times out and reports it */
    }
    memset(req, 0, sizeof(req));
    memcpy(req, data, len);
    /* Sensor access belongs on the system work queue with the trackpad driver. */
    k_work_submit(&handle_work);
    return BT_GATT_ITER_CONTINUE;
}

/* Each step stops as soon as it finds its attribute, so discovery never ends with an
 * error response (which would block this side's ATT requests, see right_relay.c). */
static uint8_t on_discover(struct bt_conn *c, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *params) {
    if (attr == NULL) {
        LOG_WRN("right link: the left half has no settings link (discovery type %u)", params->type);
        return BT_GATT_ITER_STOP;
    }
    LOG_INF("right link: found type %u handle 0x%04x", params->type, attr->handle);
    switch (params->type) {
    case BT_GATT_DISCOVER_PRIMARY:
        disc.uuid = &chrc_uuid.uuid;
        disc.start_handle = attr->handle + 1;
        disc.end_handle = ((struct bt_gatt_service_val *)attr->user_data)->end_handle;
        disc.type = BT_GATT_DISCOVER_CHARACTERISTIC;
        break;
    case BT_GATT_DISCOVER_CHARACTERISTIC:
        sub.value_handle = bt_gatt_attr_value_handle(attr);
        disc.uuid = &ccc_uuid.uuid;
        disc.start_handle = attr->handle + 2;
        disc.type = BT_GATT_DISCOVER_DESCRIPTOR;
        break;
    case BT_GATT_DISCOVER_DESCRIPTOR: {
        sub.ccc_handle = attr->handle;
        sub.notify = on_notify;
        sub.value = BT_GATT_CCC_NOTIFY;
        /* Subscribe afresh on every connection. */
        atomic_set_bit(sub.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
        int err = bt_gatt_subscribe(c, &sub);
        if (err == 0 || err == -EALREADY) {
            value_handle = sub.value_handle;
            LOG_INF("right link: ready");
        } else {
            LOG_WRN("right link: subscribe failed (%d)", err);
        }
        return BT_GATT_ITER_STOP;
    }
    default:
        return BT_GATT_ITER_STOP;
    }
    int err = bt_gatt_discover(c, &disc);
    if (err != 0) {
        LOG_WRN("right link: discovery step failed (%d)", err);
    }
    return BT_GATT_ITER_STOP;
}

static void discover(struct k_work *work) {
    LOG_INF("right link: discover (conn %d, subscribed %d)", conn != NULL, value_handle != 0);
    if (conn == NULL || value_handle != 0) {
        return;
    }
    disc = (struct bt_gatt_discover_params){
        .uuid = &svc_uuid.uuid,
        .func = on_discover,
        .start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
        .type = BT_GATT_DISCOVER_PRIMARY,
    };
    int err = bt_gatt_discover(conn, &disc);
    if (err != 0) {
        LOG_WRN("right link: discovery failed to start (%d)", err);
    }
}

/* Let ZMK finish its own setup on the new link first. */
static K_WORK_DELAYABLE_DEFINE(discover_work, discover);

static bool is_split_link(struct bt_conn *c) {
    struct bt_conn_info info;
    return bt_conn_get_info(c, &info) == 0 && info.role == BT_CONN_ROLE_PERIPHERAL;
}

/* The left half's link service requires encryption: start once the split link has it. */
static void security_changed(struct bt_conn *c, bt_security_t level, enum bt_security_err err) {
    LOG_INF("right link: security %u (err %d, split %d)", level, err, is_split_link(c));
    if (err != BT_SECURITY_ERR_SUCCESS || level < BT_SECURITY_L2 || !is_split_link(c)) {
        return;
    }
    if (conn == NULL) {
        conn = bt_conn_ref(c);
    }
    k_work_reschedule(&discover_work, K_SECONDS(2));
}

static void disconnected(struct bt_conn *c, uint8_t reason) {
    LOG_INF("right link: disconnected (0x%02x, split %d)", reason, c == conn);
    if (c != conn) {
        return;
    }
    k_work_cancel_delayable(&discover_work);
    value_handle = 0;
    bt_conn_unref(conn);
    conn = NULL;
}

BT_CONN_CB_DEFINE(toucan_studio_right_link) = {
    .security_changed = security_changed,
    .disconnected = disconnected,
};
