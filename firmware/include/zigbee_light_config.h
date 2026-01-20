/**
 * @file zigbee_light_config.h
 * @brief Zigbee LED Copper String Controller Configuration
 *
 * Defines all Zigbee clusters, attributes, and endpoint configuration
 * for a ZHA-compliant dimmable light device.
 */

#ifndef ZIGBEE_LIGHT_CONFIG_H
#define ZIGBEE_LIGHT_CONFIG_H

#include <zephyr/kernel.h>
#include <zboss_api.h>
#include <zboss_api_addons.h>
#include "zb_dimmable_light.h"

/* ==========================================================================
 * Device Identification
 * ========================================================================== */

/**
 * Manufacturer code - Registered with Zigbee Alliance or use test range.
 * Test/development range: 0x1000-0x10FF
 * Generated value within test range.
 */
#define ZB_LIGHT_MANUFACTURER_CODE      0x1042U

/**
 * Device identifiers following Zigbee HA profile specification.
 * HA_DEV_ID_DIMMABLE_LIGHT = 0x0101 per ZCL specification.
 */
#define ZB_LIGHT_DEVICE_ID              ZB_DIMMABLE_LIGHT_DEVICE_ID
#define ZB_LIGHT_DEVICE_VERSION         1U

/* ==========================================================================
 * Endpoint Configuration
 * ========================================================================== */

#define ZB_LIGHT_ENDPOINT               1U

/* ==========================================================================
 * Cluster Attribute Counts
 * ========================================================================== */

#define ZB_LIGHT_BASIC_ATTR_COUNT       (ZB_ZCL_BASIC_ATTR_COUNT + 4)
#define ZB_LIGHT_IDENTIFY_ATTR_COUNT    ZB_ZCL_IDENTIFY_ATTR_COUNT
#define ZB_LIGHT_GROUPS_ATTR_COUNT      ZB_ZCL_GROUPS_ATTR_COUNT
#define ZB_LIGHT_SCENES_ATTR_COUNT      ZB_ZCL_SCENES_ATTR_COUNT
#define ZB_LIGHT_ON_OFF_ATTR_COUNT      ZB_ZCL_ON_OFF_ATTR_COUNT
#define ZB_LIGHT_LEVEL_CONTROL_ATTR_COUNT ZB_ZCL_LEVEL_CONTROL_ATTR_COUNT

/* ==========================================================================
 * Cluster Lists
 * ========================================================================== */

#define ZB_LIGHT_IN_CLUSTER_COUNT       6U
#define ZB_LIGHT_OUT_CLUSTER_COUNT      0U
#define ZB_LIGHT_REPORT_ATTR_COUNT      2U

/* ==========================================================================
 * String Constants (ZCL max lengths enforced)
 * ========================================================================== */

#define ZB_LIGHT_MANUFACTURER_NAME      "DIY"
#define ZB_LIGHT_MANUFACTURER_NAME_LEN  3U

#define ZB_LIGHT_MODEL_ID               "LEDCopperV1"
#define ZB_LIGHT_MODEL_ID_LEN           11U

#define ZB_LIGHT_DATE_CODE              "20250120"
#define ZB_LIGHT_DATE_CODE_LEN          8U

#define ZB_LIGHT_SW_BUILD_ID            "1.0.0"
#define ZB_LIGHT_SW_BUILD_ID_LEN        5U

/* ==========================================================================
 * Level Control Configuration
 * ========================================================================== */

#define ZB_LIGHT_LEVEL_MIN              0U
#define ZB_LIGHT_LEVEL_MAX              254U
#define ZB_LIGHT_LEVEL_DEFAULT          254U
#define ZB_LIGHT_TRANSITION_TIME        5U

/* ==========================================================================
 * TB6612 Configuration
 * ========================================================================== */

/* PWM frequency for brightness control (1kHz default) */
#define TB6612_PWM_FREQUENCY_HZ         1000U
#define TB6612_PWM_PERIOD_US            (1000000U / TB6612_PWM_FREQUENCY_HZ)

/* Polarity alternation frequency (how fast to switch between halves) */
#ifdef CONFIG_APP_TB6612_POLARITY_FREQ_HZ
#define TB6612_POLARITY_FREQ_HZ         CONFIG_APP_TB6612_POLARITY_FREQ_HZ
#else
#define TB6612_POLARITY_FREQ_HZ         100U
#endif

#define TB6612_POLARITY_PERIOD_US       (1000000U / TB6612_POLARITY_FREQ_HZ)

/* ==========================================================================
 * Button Configuration
 * ========================================================================== */

#define BUTTON_LONG_PRESS_THRESHOLD_MS  3000U
#define BUTTON_DEBOUNCE_TIME_MS         50U

/* ==========================================================================
 * Network Configuration
 * ========================================================================== */

#define ZB_LIGHT_STEERING_TIMEOUT_SEC   180U

/* ==========================================================================
 * Install Code (16 bytes + 2 byte CRC)
 * Generated for development - replace in production
 * ========================================================================== */

#define ZB_LIGHT_INSTALL_CODE { \
    0x83U, 0xFEU, 0xD3U, 0x40U, 0x7AU, 0x93U, 0x97U, 0x23U, \
    0xA5U, 0xC6U, 0x39U, 0xB2U, 0x69U, 0x16U, 0xD5U, 0x05U, \
    0xC3U, 0xB5U  /* CRC16 */ \
}

/* ==========================================================================
 * Conversion Macros
 * ========================================================================== */

/**
 * Convert Zigbee level (0-254) to PWM duty cycle.
 */
#define ZB_LEVEL_TO_PWM(level, period) \
    (((uint32_t)(level) * (uint32_t)(period)) / 254U)

/**
 * Convert PWM duty cycle to Zigbee level.
 */
#define PWM_TO_ZB_LEVEL(duty, period) \
    ((uint8_t)(((uint32_t)(duty) * 254U) / (uint32_t)(period)))

#endif /* ZIGBEE_LIGHT_CONFIG_H */
