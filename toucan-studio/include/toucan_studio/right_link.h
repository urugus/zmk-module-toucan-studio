#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/uuid.h>

/* A15 (D25): the settings link service. It lives on the left half; the right half subscribes
 * to it. Requests: left -> right notifications. Replies: right -> left writes without response. */
#define TSP_RIGHT_SERVICE_UUID BT_UUID_128_ENCODE(0x439d285e, 0xc04c, 0x4298, 0x8c3e, 0x66a8ccc66d45)
#define TSP_RIGHT_CHRC_UUID BT_UUID_128_ENCODE(0xe960e884, 0x60b9, 0x40c8, 0xa44a, 0x5886371e38b5)

/* A14 frames carried unchanged: request [ver, seq, cmd, field, value],
 * reply [ver, seq, status, flags, cur(3), def(3)]. */
#define TSP_RIGHT_REQ_LEN 5
#define TSP_RIGHT_REPLY_LEN 10
/* A21 (M8): get_right_build replies [ver, seq, status, side, id(8)]; right_enter_dfu requests
 * [ver, seq, cmd, 'B', 'O', 'O', 'T']. Settings frames keep their lengths, so a right half
 * without A21 ignores the longer request and the left half reports "no reply". */
#define TSP_RIGHT_REQ_MAX 7
#define TSP_RIGHT_REPLY_MAX 12
#define TSP_CMD_RIGHT_BUILD 0x35
#define TSP_CMD_RIGHT_DFU 0x36

#define TSP_STATUS_OK 0
#define TSP_STATUS_UNKNOWN_CMD 1
#define TSP_STATUS_OUT_OF_RANGE 2
#define TSP_STATUS_STORAGE 3
#define TSP_STATUS_BAD_VERSION 4
#define TSP_STATUS_RIGHT_NOT_CONNECTED 5
#define TSP_STATUS_RIGHT_NO_REPLY 6
#define TSP_STATUS_SENSOR 7 /* the right half could not write its sensor */

#define TSP_CMD_RIGHT_FIRST 0x30
#define TSP_CMD_RIGHT_LAST 0x36

/* Left half (central): forwards a request to the right half. Returns -ENOTCONN until the
 * right half has subscribed to the link. The reply arrives through `on_reply`. */
/* `reply` is NULL (len 0) when the request cannot be answered (link lost, write failed). */
typedef void (*tsp_right_reply_cb)(const uint8_t *reply, size_t len);
int tsp_right_relay_send(const uint8_t *req, size_t len, tsp_right_reply_cb on_reply);
bool tsp_right_relay_ready(void);
/* Diagnostics for the "not connected" reply: [stage, security level, security events]
 * (stages in right_relay.c). */
#define TSP_RIGHT_DIAG_LEN 3
void tsp_right_relay_diag(uint8_t out[TSP_RIGHT_DIAG_LEN]);
/* Drops an unanswered request (after the caller's timeout). */
void tsp_right_relay_cancel(void);
