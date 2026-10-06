/*
 * Left half (central): relays right-half settings requests over the split link (M6, A14/A15).
 *
 * The service lives here and the right half is its GATT client (A15, D25). The left half
 * never issues ATT requests to the right half: with the Zephyr 3.5 used by ZMK, any error
 * response (ZMK's own split discovery ends with one) leaves the central's request queue
 * blocked for the rest of the connection. Requests go out as notifications; replies come
 * back as writes without response.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <toucan_studio/right_link.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* Where link preparation is, reported with "right half not connected" (diagnostics). */
enum stage {
    ST_NO_LINK = 1,       /* no split connection */
    ST_NOT_SUBSCRIBED = 2, /* connected, the right half has not subscribed yet */
    ST_READY = 10,
};
static uint8_t security_events; /* how often the split link reported encryption */
static tsp_right_reply_cb pending_cb; /* set while a request waits for its reply */

static bool is_split_link(struct bt_conn *c) {
    struct bt_conn_info info;
    return bt_conn_get_info(c, &info) == 0 && info.role == BT_CONN_ROLE_CENTRAL;
}

static ssize_t on_write(struct bt_conn *c, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
                        uint16_t offset, uint8_t flags) {
    if (!is_split_link(c)) {
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED); /* only the right half answers */
    }
    if (offset != 0 || len == 0 || len > TSP_RIGHT_REPLY_MAX) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    tsp_right_reply_cb cb = pending_cb;
    pending_cb = NULL;
    if (cb) {
        cb(buf, len);
    }
    return len;
}

BT_GATT_SERVICE_DEFINE(tsp_link_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(TSP_RIGHT_SERVICE_UUID)),
                       BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(TSP_RIGHT_CHRC_UUID),
                                              BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_WRITE_ENCRYPT, NULL, on_write, NULL),
                       BT_GATT_CCC(NULL, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT));

#define VALUE_ATTR (&tsp_link_svc.attrs[2])

static void find_split_link(struct bt_conn *c, void *data) {
    struct bt_conn **out = data;
    if (*out == NULL && is_split_link(c)) {
        *out = bt_conn_ref(c);
    }
}

/* The split link with a reference, or NULL. */
static struct bt_conn *split_link(void) {
    struct bt_conn *c = NULL;
    bt_conn_foreach(BT_CONN_TYPE_LE, find_split_link, &c);
    return c;
}

static uint8_t current_stage(void) {
    struct bt_conn *c = split_link();
    if (c == NULL) {
        return ST_NO_LINK;
    }
    bool subscribed = bt_gatt_is_subscribed(c, VALUE_ATTR, BT_GATT_CCC_NOTIFY);
    bt_conn_unref(c);
    return subscribed ? ST_READY : ST_NOT_SUBSCRIBED;
}

void tsp_right_relay_diag(uint8_t out[TSP_RIGHT_DIAG_LEN]) {
    struct bt_conn *c = split_link();
    out[0] = current_stage();
    out[1] = c ? bt_conn_get_security(c) : 0;
    out[2] = security_events;
    if (c) {
        bt_conn_unref(c);
    }
}

bool tsp_right_relay_ready(void) { return current_stage() == ST_READY; }

int tsp_right_relay_send(const uint8_t *req, size_t len, tsp_right_reply_cb on_reply) {
    struct bt_conn *c = split_link();
    if (c == NULL) {
        return -ENOTCONN;
    }
    int err;
    if (!bt_gatt_is_subscribed(c, VALUE_ATTR, BT_GATT_CCC_NOTIFY)) {
        err = -ENOTCONN;
    } else if (pending_cb != NULL) {
        err = -EBUSY;
    } else {
        pending_cb = on_reply;
        err = bt_gatt_notify(c, VALUE_ATTR, req, len);
        if (err) {
            pending_cb = NULL;
        }
    }
    bt_conn_unref(c);
    return err;
}

void tsp_right_relay_cancel(void) { pending_cb = NULL; }

static void security_changed(struct bt_conn *c, bt_security_t level, enum bt_security_err err) {
    if (is_split_link(c)) {
        security_events++;
    }
}

static void disconnected(struct bt_conn *c, uint8_t reason) {
    if (!is_split_link(c)) {
        return;
    }
    tsp_right_reply_cb cb = pending_cb;
    pending_cb = NULL;
    if (cb) {
        cb(NULL, 0); /* the request can no longer be answered */
    }
}

BT_CONN_CB_DEFINE(toucan_studio_right_relay) = {
    .security_changed = security_changed,
    .disconnected = disconnected,
};
