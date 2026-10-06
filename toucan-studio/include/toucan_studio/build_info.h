#pragma once

#include <stdint.h>

/*
 * Build identity embedded in the image (A19). The app finds it by scanning the UF2 for the
 * magic, and the left half reports it over raw HID (A18 get_build).
 */
#define TSP_BUILD_INFO_MAGIC "TSFWINF"
#define TSP_BUILD_INFO_FMT 1
#define TSP_BUILD_ID_LEN 8
#define TSP_BUILD_LABEL_LEN 20

enum tsp_side {
    TSP_SIDE_LEFT = 1,
    TSP_SIDE_RIGHT = 2,
};

struct tsp_build_info {
    char magic[7];
    uint8_t fmt;
    uint8_t side;
    uint8_t reserved[3];
    /* Random per build (firmware/build.sh); all zero when the build did not set one. */
    uint8_t id[TSP_BUILD_ID_LEN];
    /* ASCII, zero padded, not necessarily terminated. */
    char label[TSP_BUILD_LABEL_LEN];
} __packed;

extern const struct tsp_build_info tsp_build_info;
