/*
 * Build identity (A19). TSP_BUILD_ID / TSP_BUILD_LABEL come from the header that
 * CMakeLists.txt generates from -DTOUCAN_BUILD_ID / -DTOUCAN_BUILD_LABEL.
 */
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <toucan_studio/build_info.h>
#include <tsp_build_info_values.h>

BUILD_ASSERT(sizeof(struct tsp_build_info) == 40, "A19 fixes the block at 40 bytes");
BUILD_ASSERT(sizeof(TSP_BUILD_LABEL) - 1 <= TSP_BUILD_LABEL_LEN, "build label too long");

const struct tsp_build_info tsp_build_info = {
    .magic = {'T', 'S', 'F', 'W', 'I', 'N', 'F'},
    .fmt = TSP_BUILD_INFO_FMT,
    .side = IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) ? TSP_SIDE_LEFT : TSP_SIDE_RIGHT,
    .id = TSP_BUILD_ID,
    .label = TSP_BUILD_LABEL,
};

/* The right half never reads the block; this reference keeps --gc-sections from dropping it. */
static int keep_build_info(void) {
    __asm__ volatile("" : : "r"(&tsp_build_info) : "memory");
    return 0;
}

SYS_INIT(keep_build_info, APPLICATION, 99);
