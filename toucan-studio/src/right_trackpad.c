/*
 * Right-half trackpad settings (M6, A14/A16/D24).
 *
 * The Cirque Pinnacle driver is used unchanged. This file writes the sensor's registers
 * directly over the same SPI device. Register addresses and the extended-register (ERA)
 * procedure follow Cirque's published register map, as also used by Zephyr's
 * Apache-2.0 `drivers/input/input_pinnacle.c`. No code is copied from the (unlicensed)
 * cirque-input-module driver.
 *
 * Every function here runs on the system work queue, the same queue the driver reads
 * the sensor from, so register sequences never interleave with its packet reads.
 *
 * The driver's data-ready interrupt is masked while a sequence runs and re-armed after
 * it: the sequence raises the sensor's status flags, and when the driver's handler then
 * finds no motion data it returns without re-arming its interrupt, which stops the
 * trackpad until a reboot (seen on hardware, M6.5).
 */
#define DT_DRV_COMPAT zmk_toucan_right_trackpad

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <toucan_studio/right_trackpad.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "exactly one toucan right trackpad node is supported");

#define PAD DT_INST_PHANDLE(0, trackpad)

/* ---- register map (Cirque Pinnacle) --------------------------------------------------- */
#define REG_STATUS1 0x02
#define REG_SYS_CONFIG1 0x03
#define REG_FEED_CONFIG1 0x04
#define REG_FEED_CONFIG2 0x05
#define REG_CAL_CONFIG1 0x07
#define REG_ERA_VALUE 0x1B
#define REG_ERA_ADDR_HIGH 0x1C
#define REG_ERA_ADDR_LOW 0x1D
#define REG_ERA_CTRL 0x1E

#define SYS_CONFIG1_SLEEP BIT(2)
#define FEED1_ENABLE BIT(0)
#define FEED2_DISABLE_TAPS BIT(1)
#define CAL1_CALIBRATE BIT(0)

#define ERA_TRACKING_ADC_CONFIG 0x0187 /* bits 7:6 = ADC attenuation */
#define ERA_X_AXIS_WIDE_Z_MIN 0x0149
#define ERA_Y_AXIS_WIDE_Z_MIN 0x0168
#define ERA_CTRL_READ BIT(0)
#define ERA_CTRL_WRITE BIT(1)

#define SPI_READ(a) (0xA0 | (a))
#define SPI_WRITE(a) (0x80 | (a))
#define SPI_FILLER 0xFB

static const struct spi_dt_spec spi =
    SPI_DT_SPEC_GET(PAD, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA, 0);

static const struct gpio_dt_spec dr = GPIO_DT_SPEC_GET(PAD, dr_gpios);

static int reg_write(uint8_t addr, uint8_t value) {
    uint8_t tx[] = {SPI_WRITE(addr), value};
    const struct spi_buf buf = {.buf = tx, .len = sizeof(tx)};
    const struct spi_buf_set set = {.buffers = &buf, .count = 1};
    return spi_write_dt(&spi, &set);
}

static int reg_read(uint8_t addr, uint8_t *value) {
    uint8_t tx[] = {SPI_READ(addr), SPI_FILLER, SPI_FILLER, SPI_FILLER};
    uint8_t rx[sizeof(tx)];
    const struct spi_buf txb = {.buf = tx, .len = sizeof(tx)};
    const struct spi_buf rxb = {.buf = rx, .len = sizeof(rx)};
    const struct spi_buf_set txs = {.buffers = &txb, .count = 1};
    const struct spi_buf_set rxs = {.buffers = &rxb, .count = 1};
    int err = spi_transceive_dt(&spi, &txs, &rxs);
    if (err == 0) {
        *value = rx[3];
    }
    return err;
}

/* Polls `reg` until (value & mask) == want, up to about `tries` x 10 ms. */
static int wait_bits(uint8_t reg, uint8_t mask, uint8_t want, int tries) {
    for (int i = 0; i < tries; i++) {
        uint8_t v;
        int err = reg_read(reg, &v);
        if (err) {
            return err;
        }
        if ((v & mask) == want) {
            return 0;
        }
        k_msleep(10);
    }
    return -ETIMEDOUT;
}

static int era_write(uint16_t addr, uint8_t value) {
    int err = reg_write(REG_ERA_VALUE, value);
    err = err ?: reg_write(REG_ERA_ADDR_HIGH, addr >> 8);
    err = err ?: reg_write(REG_ERA_ADDR_LOW, addr & 0xFF);
    err = err ?: reg_write(REG_ERA_CTRL, ERA_CTRL_WRITE);
    return err ?: wait_bits(REG_ERA_CTRL, 0xFF, 0x00, 5);
}

static int era_read(uint16_t addr, uint8_t *value) {
    int err = reg_write(REG_ERA_ADDR_HIGH, addr >> 8);
    err = err ?: reg_write(REG_ERA_ADDR_LOW, addr & 0xFF);
    err = err ?: reg_write(REG_ERA_CTRL, ERA_CTRL_READ);
    err = err ?: wait_bits(REG_ERA_CTRL, 0xFF, 0x00, 5);
    return err ?: reg_read(REG_ERA_VALUE, value);
}

static int update_bits(uint8_t reg, uint8_t mask, uint8_t bits) {
    uint8_t v;
    int err = reg_read(reg, &v);
    return err ?: reg_write(reg, (v & ~mask) | (bits & mask));
}

/* ---- settings -------------------------------------------------------------------------- */

#define DEFAULT_VALUES                                                                                         \
    {                                                                                                          \
        .bits = (DT_PROP(PAD, no_taps) ? 0 : TSP_RIGHT_TAP) |                                                        \
            (DT_PROP(PAD, sleep) ? TSP_RIGHT_SLEEP : 0),                                                     \
        /* The driver's `sensitivity` enum index 0..3 = 1x..4x (driver default 1x). */                     \
        .sens = DT_ENUM_IDX_OR(PAD, sensitivity, 0) + 1, .z = DT_PROP(PAD, y_axis_z_min),                 \
    }

/* The X threshold keeps the devicetree's distance from the Y threshold (D27). */
#define Z_X_OFFSET (DT_PROP(PAD, x_axis_z_min) - DT_PROP(PAD, y_axis_z_min))
BUILD_ASSERT(Z_X_OFFSET >= 0 && TSP_RIGHT_Z_MAX + Z_X_OFFSET <= 16, "unsupported x/y-axis-z-min pair");
BUILD_ASSERT(DT_PROP(PAD, y_axis_z_min) >= TSP_RIGHT_Z_MIN && DT_PROP(PAD, y_axis_z_min) <= TSP_RIGHT_Z_MAX,
             "y-axis-z-min outside the settable range");

static const struct tsp_right_values def = DEFAULT_VALUES;
/* What the sensor has: the driver's devicetree values until something is applied. */
static struct tsp_right_values cur = DEFAULT_VALUES;
static struct tsp_right_values saved_vals;
static bool has_saved;

static bool values_ok(const struct tsp_right_values *v) {
    return (v->bits & ~TSP_RIGHT_BITS_MASK) == 0 && v->sens >= TSP_RIGHT_SENS_MIN && v->sens <= TSP_RIGHT_SENS_MAX &&
           v->z >= TSP_RIGHT_Z_MIN && v->z <= TSP_RIGHT_Z_MAX;
}

static bool values_equal(const struct tsp_right_values *a, const struct tsp_right_values *b) {
    return a->bits == b->bits && a->sens == b->sens && a->z == b->z;
}

/* Writes what differs between `from` (what the sensor has) and `to`. */
static int apply(const struct tsp_right_values *from, const struct tsp_right_values *to) {
    uint8_t feed1;
    int err = reg_read(REG_FEED_CONFIG1, &feed1);
    if (err) {
        return err;
    }
    bool adc = from->sens != to->sens || from->z != to->z;
    if (adc) {
        /* Extended registers are accessed with the data feed paused. */
        err = reg_write(REG_FEED_CONFIG1, feed1 & ~FEED1_ENABLE);
        uint8_t v = 0;
        err = err ?: era_read(ERA_TRACKING_ADC_CONFIG, &v);
        err = err ?: era_write(ERA_TRACKING_ADC_CONFIG, (v & 0x3F) | ((to->sens - 1) << 6));
        err = err ?: era_write(ERA_X_AXIS_WIDE_Z_MIN, to->z + Z_X_OFFSET);
        err = err ?: era_write(ERA_Y_AXIS_WIDE_Z_MIN, to->z);
        /* Recalibrate for the new ADC settings; the bit clears when done. */
        err = err ?: update_bits(REG_CAL_CONFIG1, CAL1_CALIBRATE, CAL1_CALIBRATE);
        err = err ?: wait_bits(REG_CAL_CONFIG1, CAL1_CALIBRATE, 0, 20);
        if (err) {
            reg_write(REG_FEED_CONFIG1, feed1); /* best effort: resume reporting */
            return err;
        }
    }
    err = update_bits(REG_FEED_CONFIG2, FEED2_DISABLE_TAPS, to->bits & TSP_RIGHT_TAP ? 0 : FEED2_DISABLE_TAPS);
    err = err ?: update_bits(REG_SYS_CONFIG1, SYS_CONFIG1_SLEEP, to->bits & TSP_RIGHT_SLEEP ? SYS_CONFIG1_SLEEP : 0);
    err = err ?: reg_write(REG_FEED_CONFIG1, feed1 | FEED1_ENABLE);
    reg_write(REG_STATUS1, 0); /* clear flags raised by the sequence */
    return err;
}

/* Changes `cur` to `next` only if the sensor accepted it. */
static int apply_and_keep(const struct tsp_right_values *next) {
    gpio_pin_interrupt_configure_dt(&dr, GPIO_INT_DISABLE);
    int err = apply(&cur, next);
    /* A handler the driver queued before the mask finds the flags cleared and returns
     * without re-arming; arming here keeps the interrupt alive either way. The driver
     * uses a level interrupt on the pin. */
    gpio_pin_interrupt_configure_dt(&dr, GPIO_INT_LEVEL_ACTIVE);
    if (err) {
        LOG_ERR("right trackpad: sensor write failed (%d)", err);
        return -EIO;
    }
    cur = *next;
    return 0;
}

void tsp_right_get(struct tsp_right_state *out) {
    out->cur = cur;
    out->def = def;
    out->saved = has_saved;
    out->dirty = !values_equal(&cur, has_saved ? &saved_vals : &def);
}

int tsp_right_set(uint8_t field, uint8_t value) {
    struct tsp_right_values next = cur;
    uint8_t bit = 0;
    switch (field) {
    case TSP_RIGHT_F_SENS:
        next.sens = value;
        break;
    case TSP_RIGHT_F_Z:
        next.z = value;
        break;
    case TSP_RIGHT_F_TAP:
        bit = TSP_RIGHT_TAP;
        break;
    case TSP_RIGHT_F_SLEEP:
        bit = TSP_RIGHT_SLEEP;
        break;
    default:
        return -EINVAL;
    }
    if (bit) {
        if (value > 1) {
            return -EINVAL;
        }
        next.bits = value ? (next.bits | bit) : (next.bits & ~bit);
    }
    if (!values_ok(&next)) {
        return -EINVAL;
    }
    return apply_and_keep(&next);
}

#define SETTINGS_KEY "tsp/right"
/* A16 (D27): [ver=2, bits, sens, z]. Version 1 was [1, bits, sens, zx, zy]. */
#define BLOB_VERSION 2

int tsp_right_save(void) {
    uint8_t b[] = {BLOB_VERSION, cur.bits, cur.sens, cur.z};
    int err = settings_save_one(SETTINGS_KEY, b, sizeof(b));
    if (err) {
        LOG_ERR("right trackpad: save failed (%d)", err);
        return err;
    }
    saved_vals = cur;
    has_saved = true;
    return 0;
}

int tsp_right_revert(void) { return apply_and_keep(has_saved ? &saved_vals : &def); }

int tsp_right_reset(void) {
    int err = settings_delete(SETTINGS_KEY);
    if (err) {
        LOG_ERR("right trackpad: delete failed (%d)", err);
        return err;
    }
    has_saved = false;
    return apply_and_keep(&def);
}

/* The driver configures the sensor from its devicetree values at boot. Saved values
 * are applied afterwards on the system work queue. */
static void apply_saved(struct k_work *work) {
    if (has_saved && !values_equal(&cur, &saved_vals)) {
        apply_and_keep(&saved_vals);
    }
}

static K_WORK_DELAYABLE_DEFINE(apply_saved_work, apply_saved);

static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    if (name != NULL && name[0] != '\0') {
        return -ENOENT;
    }
    uint8_t b[5];
    if (len < 4 || len > sizeof(b) || read_cb(cb_arg, b, len) != len ||
        !((b[0] == BLOB_VERSION && len == 4) || (b[0] == 1 && len == 5))) {
        LOG_WRN("right trackpad: ignoring stored settings");
        return 0;
    }
    /* Version 1 kept separate thresholds: its Y value becomes z (the next save writes
     * version 2). Removed bits (D26, D28) may remain in values saved before. */
    struct tsp_right_values v = {.bits = b[1] & TSP_RIGHT_BITS_MASK, .sens = b[2], .z = b[0] == 1 ? b[4] : b[3]};
    if (!values_ok(&v)) {
        LOG_WRN("right trackpad: ignoring out-of-range settings");
        return 0;
    }
    saved_vals = v;
    has_saved = true;
    /* Give the driver time to finish its own initialisation first. */
    k_work_schedule(&apply_saved_work, K_MSEC(500));
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(tsp_right, SETTINGS_KEY, NULL, settings_set, NULL, NULL);

static int init(void) {
    if (!spi_is_ready_dt(&spi)) {
        LOG_ERR("right trackpad: SPI not ready");
    }
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
