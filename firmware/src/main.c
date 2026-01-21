/**
 * @file main.c
 * @brief LED Copper String Zigbee Controller
 *
 * Controls a special LED strip via TB6612 H-bridge where:
 * - One polarity lights half the strip
 * - Reverse polarity lights the other half
 * - Rapid polarity alternation makes both halves appear lit
 *
 * TB6612 Wiring:
 * - PWMA:    P0.22 (brightness via PWM)
 * - AIN1:    P0.20 (polarity control)
 * - AIN2:    P0.17 (polarity control)
 * - STANDBY: P0.24 (HIGH=active, LOW=standby)
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <hal/nrf_saadc.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zb_nrf_platform.h>
#include <zcl/zb_zcl_power_config.h>
#include "zb_dimmable_light.h"

#ifdef CONFIG_ZIGBEE_FOTA
#include <zigbee/zigbee_fota.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>
#endif

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ==========================================================================
 * Configuration
 * ========================================================================== */

#define LIGHT_ENDPOINT                  1

#define BULB_INIT_BASIC_APP_VERSION     1
#define BULB_INIT_BASIC_STACK_VERSION   1
#define BULB_INIT_BASIC_HW_VERSION      1
#define BULB_INIT_BASIC_MANUF_NAME      "DIY"
#define BULB_INIT_BASIC_MODEL_ID        "LEDCopperV1"
#define BULB_INIT_BASIC_DATE_CODE       "20260120"
#define BULB_INIT_BASIC_LOCATION_DESC   ""
#define BULB_INIT_BASIC_PH_ENV          ZB_ZCL_BASIC_ENV_UNSPECIFIED

#define BUTTON_LONG_PRESS_MS            3000

/* Startup behavior values for On/Off cluster */
#define ZB_ZCL_ON_OFF_STARTUP_OFF       0x00
#define ZB_ZCL_ON_OFF_STARTUP_ON        0x01
#define ZB_ZCL_ON_OFF_STARTUP_TOGGLE    0x02
#define ZB_ZCL_ON_OFF_STARTUP_PREVIOUS  0xFF

/* Startup behavior values for Level Control cluster */
#define ZB_ZCL_LEVEL_STARTUP_MINIMUM    0x00
#define ZB_ZCL_LEVEL_STARTUP_PREVIOUS   0xFF

/* TB6612 polarity alternation period (microseconds) */
#ifdef CONFIG_APP_TB6612_POLARITY_FREQ_HZ
#define POLARITY_PERIOD_US              (1000000U / CONFIG_APP_TB6612_POLARITY_FREQ_HZ)
#else
#define POLARITY_PERIOD_US              10000U  /* 100Hz default */
#endif

/* Battery measurement configuration */
#ifdef CONFIG_APP_BATTERY_REPORT_INTERVAL_SEC
#define BATTERY_REPORT_INTERVAL_SEC     CONFIG_APP_BATTERY_REPORT_INTERVAL_SEC
#else
#define BATTERY_REPORT_INTERVAL_SEC     3600U   /* 1 hour default */
#endif

/* Battery endpoint - use same endpoint as light for simplicity */
#define BATTERY_ENDPOINT                LIGHT_ENDPOINT

/* ==========================================================================
 * Device Tree
 * ========================================================================== */

/* PWM for brightness (PWMA) */
static const struct pwm_dt_spec pwm_brightness = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));

/* TB6612 control pins */
static const struct gpio_dt_spec tb6612_ain1 = GPIO_DT_SPEC_GET(DT_NODELABEL(tb6612_ain1), gpios);
static const struct gpio_dt_spec tb6612_ain2 = GPIO_DT_SPEC_GET(DT_NODELABEL(tb6612_ain2), gpios);
static const struct gpio_dt_spec tb6612_standby = GPIO_DT_SPEC_GET(DT_NODELABEL(tb6612_standby), gpios);

/* Button and status LED */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* ==========================================================================
 * Application Context
 * ========================================================================== */

/* Extended On/Off attributes including startup behavior */
typedef struct {
	zb_bool_t   on_off;
	zb_bool_t   global_scene_ctrl;
	zb_uint16_t on_time;
	zb_uint16_t off_wait_time;
	zb_uint8_t  start_up_on_off;       /* Power-on behavior: 0=off, 1=on, 2=toggle, 0xFF=previous */
} on_off_attrs_ext_t;

/* Extended Level Control attributes including startup behavior and transition time */
typedef struct {
	zb_uint8_t  current_level;
	zb_uint16_t remaining_time;
	zb_uint8_t  options;
	zb_uint16_t on_off_transition_time; /* Transition time in 1/10th seconds */
	zb_uint8_t  start_up_current_level; /* Startup level: 0=min, 0xFF=previous, other=specific */
} level_control_attrs_ext_t;

/* Power Configuration cluster attributes for battery */
typedef struct {
	zb_uint8_t  battery_voltage;          /* In units of 100mV */
	zb_uint8_t  battery_percentage;       /* 0-200 (0.5% per unit, 200 = 100%) */
	zb_uint8_t  battery_size;
	zb_uint16_t battery_quantity;
	zb_uint8_t  battery_rated_voltage;    /* In units of 100mV */
	zb_uint8_t  battery_alarm_mask;
	zb_uint8_t  battery_voltage_min_threshold;
} power_config_attrs_t;

typedef struct {
	zb_zcl_basic_attrs_ext_t     basic_attr;
	zb_zcl_identify_attrs_t      identify_attr;
	zb_zcl_scenes_attrs_t        scenes_attr;
	zb_zcl_groups_attrs_t        groups_attr;
	on_off_attrs_ext_t           on_off_attr;
	level_control_attrs_ext_t    level_control_attr;
	power_config_attrs_t         power_config_attr;
} light_device_ctx_t;

static light_device_ctx_t dev_ctx;

/* Button state */
static struct {
	int64_t press_time;
	bool    pressed;
	uint8_t last_brightness;
} app_state = {
	.last_brightness = 254,
};

static struct gpio_callback button_cb_data;
static struct k_work button_work;
static struct k_work_delayable long_press_work;

/* Effect state */
static struct k_work_delayable effect_work;
static uint8_t effect_type;
static uint8_t effect_step;

/* TB6612 polarity alternation state */
static struct k_timer polarity_timer;
static volatile bool polarity_phase;  /* false=AIN1 high, true=AIN2 high */
static volatile bool light_is_on;

/* Battery measurement state */
static struct k_work_delayable battery_work;
static const struct device *adc_dev;

/* ==========================================================================
 * TB6612 H-Bridge Control
 * ========================================================================== */

/**
 * Timer callback for polarity alternation.
 * Switches between AIN1 high and AIN2 high to light both LED halves.
 */
static void polarity_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	if (!light_is_on) {
		return;
	}

	polarity_phase = !polarity_phase;

	if (polarity_phase) {
		/* Phase B: AIN1=LOW, AIN2=HIGH */
		gpio_pin_set_dt(&tb6612_ain1, 0);
		gpio_pin_set_dt(&tb6612_ain2, 1);
	} else {
		/* Phase A: AIN1=HIGH, AIN2=LOW */
		gpio_pin_set_dt(&tb6612_ain1, 1);
		gpio_pin_set_dt(&tb6612_ain2, 0);
	}
}

/**
 * Initialize TB6612 GPIO pins.
 */
static int tb6612_init(void)
{
	int ret;

	if (!device_is_ready(tb6612_ain1.port)) {
		LOG_ERR("TB6612 AIN1 GPIO not ready");
		return -ENODEV;
	}

	if (!device_is_ready(tb6612_ain2.port)) {
		LOG_ERR("TB6612 AIN2 GPIO not ready");
		return -ENODEV;
	}

	if (!device_is_ready(tb6612_standby.port)) {
		LOG_ERR("TB6612 STANDBY GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&tb6612_ain1, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("TB6612 AIN1 config failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&tb6612_ain2, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("TB6612 AIN2 config failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&tb6612_standby, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("TB6612 STANDBY config failed: %d", ret);
		return ret;
	}

	/* Initialize polarity alternation timer */
	k_timer_init(&polarity_timer, polarity_timer_handler, NULL);

	LOG_INF("TB6612 initialized (AIN1=P0.%u, AIN2=P0.%u, STBY=P0.%u)",
		tb6612_ain1.pin, tb6612_ain2.pin, tb6612_standby.pin);

	return 0;
}

/**
 * Turn on the TB6612 and start polarity alternation.
 */
static void tb6612_on(void)
{
	/* Enable standby (active high) */
	gpio_pin_set_dt(&tb6612_standby, 1);

	/* Start with phase A */
	polarity_phase = false;
	gpio_pin_set_dt(&tb6612_ain1, 1);
	gpio_pin_set_dt(&tb6612_ain2, 0);

	light_is_on = true;

	/* Start polarity alternation timer */
	k_timer_start(&polarity_timer, K_USEC(POLARITY_PERIOD_US / 2),
		      K_USEC(POLARITY_PERIOD_US / 2));

	LOG_DBG("TB6612 ON, polarity alternation at %u Hz",
		1000000U / POLARITY_PERIOD_US);
}

/**
 * Turn off the TB6612 and stop polarity alternation.
 */
static void tb6612_off(void)
{
	light_is_on = false;

	/* Stop polarity alternation */
	k_timer_stop(&polarity_timer);

	/* Set both AIN pins low (brake mode) */
	gpio_pin_set_dt(&tb6612_ain1, 0);
	gpio_pin_set_dt(&tb6612_ain2, 0);

	/* Disable standby for power saving */
	gpio_pin_set_dt(&tb6612_standby, 0);

	LOG_DBG("TB6612 OFF (standby)");
}

/* ==========================================================================
 * Persistent Settings - Save/restore light state across power cycles
 * ========================================================================== */

static int light_settings_set(const char *name, size_t len,
			      settings_read_cb read_cb, void *cb_arg)
{
	if (!strcmp(name, "on_off")) {
		if (len != sizeof(dev_ctx.on_off_attr.on_off)) {
			return -EINVAL;
		}
		read_cb(cb_arg, &dev_ctx.on_off_attr.on_off, len);
		LOG_INF("Restored on_off: %d", dev_ctx.on_off_attr.on_off);
	} else if (!strcmp(name, "level")) {
		if (len != sizeof(dev_ctx.level_control_attr.current_level)) {
			return -EINVAL;
		}
		read_cb(cb_arg, &dev_ctx.level_control_attr.current_level, len);
		LOG_INF("Restored level: %d", dev_ctx.level_control_attr.current_level);
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(light, "light", NULL, light_settings_set, NULL, NULL);

static void save_light_state(void)
{
	settings_save_one("light/on_off", &dev_ctx.on_off_attr.on_off,
			  sizeof(dev_ctx.on_off_attr.on_off));
	settings_save_one("light/level", &dev_ctx.level_control_attr.current_level,
			  sizeof(dev_ctx.level_control_attr.current_level));
}

/* ==========================================================================
 * Zigbee Cluster Declarations
 * ========================================================================== */

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(
	groups_attr_list,
	&dev_ctx.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(
	scenes_attr_list,
	&dev_ctx.scenes_attr.scene_count,
	&dev_ctx.scenes_attr.current_scene,
	&dev_ctx.scenes_attr.current_group,
	&dev_ctx.scenes_attr.scene_valid,
	&dev_ctx.scenes_attr.name_support);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(
	basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.app_version,
	&dev_ctx.basic_attr.stack_version,
	&dev_ctx.basic_attr.hw_version,
	dev_ctx.basic_attr.mf_name,
	dev_ctx.basic_attr.model_id,
	dev_ctx.basic_attr.date_code,
	&dev_ctx.basic_attr.power_source,
	dev_ctx.basic_attr.location_id,
	&dev_ctx.basic_attr.ph_env,
	dev_ctx.basic_attr.sw_ver);

/* On/Off attribute list - custom with startup behavior */
ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(on_off_attr_list, ZB_ZCL_ON_OFF)
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, (&dev_ctx.on_off_attr.on_off))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ON_OFF_GLOBAL_SCENE_CONTROL, (&dev_ctx.on_off_attr.global_scene_ctrl))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ON_OFF_ON_TIME, (&dev_ctx.on_off_attr.on_time))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ON_OFF_OFF_WAIT_TIME, (&dev_ctx.on_off_attr.off_wait_time))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ON_OFF_START_UP_ON_OFF, (&dev_ctx.on_off_attr.start_up_on_off))
ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST;

/* Helper macro for on_off_transition_time attribute (not in SDK) */
#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_LEVEL_CONTROL_ON_OFF_TRANSITION_TIME_ID(data_ptr) \
{                                                                                            \
  ZB_ZCL_ATTR_LEVEL_CONTROL_ON_OFF_TRANSITION_TIME_ID,                                       \
  ZB_ZCL_ATTR_TYPE_U16,                                                                      \
  ZB_ZCL_ATTR_ACCESS_READ_WRITE,                                                             \
  (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),                                                        \
  (void*) data_ptr                                                                           \
}

/* Level Control attribute list - custom with transition time */
zb_zcl_level_control_move_status_t level_control_move_status;
ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(level_control_attr_list, ZB_ZCL_LEVEL_CONTROL)
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, (&dev_ctx.level_control_attr.current_level))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_REMAINING_TIME_ID, (&dev_ctx.level_control_attr.remaining_time))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_OPTIONS_ID, (&dev_ctx.level_control_attr.options))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_ON_OFF_TRANSITION_TIME_ID, (&dev_ctx.level_control_attr.on_off_transition_time))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_START_UP_CURRENT_LEVEL_ID, (&dev_ctx.level_control_attr.start_up_current_level))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_LEVEL_CONTROL_MOVE_STATUS_ID, (&level_control_move_status))
ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST;

/* Power Configuration cluster attribute list for battery (custom to include percentage) */
ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(power_config_attr_list, ZB_ZCL_POWER_CONFIG)
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID(&dev_ctx.power_config_attr.battery_voltage, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID(&dev_ctx.power_config_attr.battery_percentage, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID(&dev_ctx.power_config_attr.battery_size, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID(&dev_ctx.power_config_attr.battery_quantity, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID(&dev_ctx.power_config_attr.battery_rated_voltage, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID(&dev_ctx.power_config_attr.battery_alarm_mask, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID(&dev_ctx.power_config_attr.battery_voltage_min_threshold, ),
ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST;

/* Custom cluster list with Power Configuration - 7 clusters total */
zb_zcl_cluster_desc_t light_clusters[] = {
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_IDENTIFY,
		ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t),
		(identify_attr_list),
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_BASIC,
		ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),
		(basic_attr_list),
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_SCENES,
		ZB_ZCL_ARRAY_SIZE(scenes_attr_list, zb_zcl_attr_t),
		(scenes_attr_list),
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_GROUPS,
		ZB_ZCL_ARRAY_SIZE(groups_attr_list, zb_zcl_attr_t),
		(groups_attr_list),
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_ARRAY_SIZE(on_off_attr_list, zb_zcl_attr_t),
		(on_off_attr_list),
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
		ZB_ZCL_ARRAY_SIZE(level_control_attr_list, zb_zcl_attr_t),
		(level_control_attr_list),
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
		ZB_ZCL_ARRAY_SIZE(power_config_attr_list, zb_zcl_attr_t),
		(power_config_attr_list),
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),
};

/* Simple descriptor for dimmable light with Power Config (7 in clusters) */
ZB_DECLARE_SIMPLE_DESC(7, 0);

ZB_AF_SIMPLE_DESC_TYPE(7, 0) simple_desc_light_ep = {
	.endpoint = LIGHT_ENDPOINT,
	.app_profile_id = ZB_AF_HA_PROFILE_ID,
	.app_device_id = ZB_DIMMABLE_LIGHT_DEVICE_ID,
	.app_device_version = ZB_DEVICE_VER_DIMMABLE_LIGHT,
	.reserved = 0,
	.app_input_cluster_count = 7,
	.app_output_cluster_count = 0,
	.app_cluster_list = {
		ZB_ZCL_CLUSTER_ID_BASIC,
		ZB_ZCL_CLUSTER_ID_IDENTIFY,
		ZB_ZCL_CLUSTER_ID_SCENES,
		ZB_ZCL_CLUSTER_ID_GROUPS,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
		ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
	}
};

/* Reporting contexts */
#define LIGHT_REPORT_ATTR_COUNT (ZB_ZCL_ON_OFF_REPORT_ATTR_COUNT + ZB_ZCL_LEVEL_CONTROL_REPORT_ATTR_COUNT)
ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info_light_ep, LIGHT_REPORT_ATTR_COUNT);
ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_info_light_ep, 1);

/* Custom endpoint declaration */
ZB_AF_DECLARE_ENDPOINT_DESC(
	light_ep,
	LIGHT_ENDPOINT,
	ZB_AF_HA_PROFILE_ID,
	0,
	NULL,
	ZB_ZCL_ARRAY_SIZE(light_clusters, zb_zcl_cluster_desc_t),
	light_clusters,
	(zb_af_simple_desc_1_1_t *)&simple_desc_light_ep,
	LIGHT_REPORT_ATTR_COUNT,
	reporting_info_light_ep,
	1,
	cvc_alarm_info_light_ep);

#ifdef CONFIG_ZIGBEE_FOTA
extern zb_af_endpoint_desc_t zigbee_fota_client_ep;

ZBOSS_DECLARE_DEVICE_CTX_2_EP(
	light_ctx,
	zigbee_fota_client_ep,
	light_ep);
#else
ZBOSS_DECLARE_DEVICE_CTX_1_EP(
	light_ctx,
	light_ep);
#endif

/* ==========================================================================
 * PWM Light Control
 * ========================================================================== */

/*
 * CIE 1931 lightness correction lookup table.
 * Maps linear input (0-255) to perceptually linear PWM output (0-255).
 * Human vision perceives brightness logarithmically, so this table
 * compensates to make dimming feel smooth and linear.
 */
static const uint8_t cie1931_lut[256] = {
	  0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,
	  2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   3,   4,
	  4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   7,
	  7,   7,   7,   8,   8,   8,   8,   9,   9,   9,  10,  10,  10,  10,  11,  11,
	 11,  12,  12,  12,  13,  13,  13,  14,  14,  15,  15,  15,  16,  16,  17,  17,
	 17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  23,  24,  24,  25,
	 25,  26,  26,  27,  28,  28,  29,  29,  30,  31,  31,  32,  32,  33,  34,  34,
	 35,  36,  37,  37,  38,  39,  39,  40,  41,  42,  43,  43,  44,  45,  46,  47,
	 47,  48,  49,  50,  51,  52,  53,  54,  54,  55,  56,  57,  58,  59,  60,  61,
	 62,  63,  64,  65,  66,  67,  68,  70,  71,  72,  73,  74,  75,  76,  77,  79,
	 80,  81,  82,  83,  85,  86,  87,  88,  90,  91,  92,  94,  95,  96,  98,  99,
	100, 102, 103, 105, 106, 108, 109, 110, 112, 113, 115, 116, 118, 120, 121, 123,
	124, 126, 128, 129, 131, 132, 134, 136, 138, 139, 141, 143, 145, 146, 148, 150,
	152, 154, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181,
	183, 185, 187, 189, 191, 193, 196, 198, 200, 202, 204, 207, 209, 211, 214, 216,
	218, 220, 223, 225, 228, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252, 255,
};

/* Track actual PWM brightness for transitions */
static uint8_t current_brightness;

static void light_set_brightness(zb_uint8_t brightness)
{
	/* Apply CIE 1931 perceptual correction */
	uint8_t corrected = cie1931_lut[brightness];

	/* Calculate pulse width */
	uint32_t pulse = (uint64_t)corrected * pwm_brightness.period / 255U;

	if (pwm_set_pulse_dt(&pwm_brightness, pulse)) {
		LOG_ERR("PWM set failed");
		return;
	}

	current_brightness = brightness;

	/* Control TB6612 on/off based on brightness */
	if (brightness > 0 && !light_is_on) {
		tb6612_on();
	} else if (brightness == 0 && light_is_on) {
		tb6612_off();
	}

	LOG_DBG("Brightness: %u -> %u (pulse: %u)", brightness, corrected, pulse);
}

/* ==========================================================================
 * Smooth Brightness Transitions
 * ========================================================================== */

#define TRANSITION_STEP_MS 20 /* Update every 20ms for smooth 50Hz */

static struct k_work_delayable transition_work;
static uint8_t transition_start;
static uint8_t transition_target;
static uint16_t transition_elapsed;
static uint16_t transition_duration;

static void transition_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	transition_elapsed += TRANSITION_STEP_MS;

	if (transition_elapsed >= transition_duration) {
		/* Transition complete */
		light_set_brightness(transition_target);
	} else {
		/* Linear interpolation (use int32_t to avoid overflow) */
		int32_t diff = (int32_t)transition_target - (int32_t)transition_start;
		uint8_t current = transition_start +
			(diff * (int32_t)transition_elapsed / (int32_t)transition_duration);
		light_set_brightness(current);
		k_work_schedule(&transition_work, K_MSEC(TRANSITION_STEP_MS));
	}
}

static void light_fade_to(uint8_t target, uint16_t duration_ms)
{
	/* Cancel any ongoing transition */
	k_work_cancel_delayable(&transition_work);

	if (duration_ms == 0 || current_brightness == target) {
		/* Instant change or already at target */
		light_set_brightness(target);
		return;
	}

	/* Start from actual current PWM brightness */
	transition_start = current_brightness;
	transition_target = target;
	transition_elapsed = 0;
	transition_duration = duration_ms;

	LOG_INF("Fade: %u -> %u over %ums", transition_start, target, duration_ms);

	/* Start transition */
	k_work_schedule(&transition_work, K_NO_WAIT);
}

static void level_control_set_value(zb_uint16_t new_level)
{
	LOG_INF("Set level: %u", new_level);

	ZB_ZCL_SET_ATTRIBUTE(
		LIGHT_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
		(zb_uint8_t *)&new_level,
		ZB_FALSE);

	light_set_brightness((zb_uint8_t)new_level);

	if (new_level > 0) {
		app_state.last_brightness = (uint8_t)new_level;
	}

	save_light_state();
}

static void on_off_set_value(zb_bool_t on)
{
	LOG_INF("Set on/off: %s", on ? "ON" : "OFF");

	ZB_ZCL_SET_ATTRIBUTE(
		LIGHT_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&on,
		ZB_FALSE);

	if (on) {
		light_set_brightness(dev_ctx.level_control_attr.current_level);
	} else {
		light_set_brightness(0U);
	}

	save_light_state();
}

static void light_toggle(void)
{
	zb_bool_t new_state = !dev_ctx.on_off_attr.on_off;
	uint8_t target_level;

	if (new_state) {
		/* Turning on - use last brightness or default */
		target_level = (dev_ctx.level_control_attr.current_level > 0) ?
			dev_ctx.level_control_attr.current_level :
			app_state.last_brightness;
		if (target_level == 0) {
			target_level = 255; /* Default to full if no previous */
		}
	} else {
		/* Turning off */
		target_level = 0;
	}

	/* Update Zigbee attributes */
	ZB_ZCL_SET_ATTRIBUTE(
		LIGHT_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&new_state,
		ZB_FALSE);

	if (new_state && target_level != dev_ctx.level_control_attr.current_level) {
		ZB_ZCL_SET_ATTRIBUTE(
			LIGHT_ENDPOINT,
			ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
			ZB_ZCL_CLUSTER_SERVER_ROLE,
			ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
			(zb_uint8_t *)&target_level,
			ZB_FALSE);
	}

	/* Smooth fade using configured transition time (convert 1/10s to ms) */
	uint16_t transition_ms = dev_ctx.level_control_attr.on_off_transition_time * 100;
	if (transition_ms == 0) {
		transition_ms = 1000; /* Default 1s if not set */
	}
	light_fade_to(target_level, transition_ms);

	if (target_level > 0) {
		app_state.last_brightness = target_level;
	}

	/* Persist state for power-on restore */
	save_light_state();

	LOG_INF("Toggle: %s (level %u, fade %ums)",
		new_state ? "ON" : "OFF", target_level, transition_ms);
}

/* ==========================================================================
 * Identify Effects
 * ========================================================================== */

static void effect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	switch (effect_type) {
	case ZB_ZCL_IDENTIFY_EFFECT_ID_BLINK:
		/* Single blink: on then off */
		if (effect_step == 0) {
			light_set_brightness(255);
			effect_step = 1;
			k_work_schedule(&effect_work, K_MSEC(500));
		} else {
			/* Restore previous state */
			if (dev_ctx.on_off_attr.on_off) {
				light_set_brightness(dev_ctx.level_control_attr.current_level);
			} else {
				light_set_brightness(0);
			}
			effect_type = ZB_ZCL_IDENTIFY_EFFECT_ID_STOP;
		}
		break;

	case ZB_ZCL_IDENTIFY_EFFECT_ID_BREATHE:
		/* Breathe: fade up and down, 15 times over ~15 seconds */
		if (effect_step < 30) {
			uint8_t brightness;
			if (effect_step % 2 == 0) {
				brightness = 255;
			} else {
				brightness = 0;
			}
			light_set_brightness(brightness);
			effect_step++;
			k_work_schedule(&effect_work, K_MSEC(500));
		} else {
			/* Restore previous state */
			if (dev_ctx.on_off_attr.on_off) {
				light_set_brightness(dev_ctx.level_control_attr.current_level);
			} else {
				light_set_brightness(0);
			}
			effect_type = ZB_ZCL_IDENTIFY_EFFECT_ID_STOP;
		}
		break;

	case ZB_ZCL_IDENTIFY_EFFECT_ID_OKAY:
		/* Okay: two quick flashes */
		if (effect_step < 4) {
			uint8_t brightness = (effect_step % 2 == 0) ? 255 : 0;
			light_set_brightness(brightness);
			effect_step++;
			k_work_schedule(&effect_work, K_MSEC(200));
		} else {
			/* Restore previous state */
			if (dev_ctx.on_off_attr.on_off) {
				light_set_brightness(dev_ctx.level_control_attr.current_level);
			} else {
				light_set_brightness(0);
			}
			effect_type = ZB_ZCL_IDENTIFY_EFFECT_ID_STOP;
		}
		break;

	case ZB_ZCL_IDENTIFY_EFFECT_ID_CHANNEL_CHANGE:
		/* Channel change: bright then dim for 8 seconds */
		if (effect_step == 0) {
			light_set_brightness(255);
			effect_step = 1;
			k_work_schedule(&effect_work, K_MSEC(500));
		} else if (effect_step == 1) {
			light_set_brightness(25);
			effect_step = 2;
			k_work_schedule(&effect_work, K_MSEC(7500));
		} else {
			/* Restore previous state */
			if (dev_ctx.on_off_attr.on_off) {
				light_set_brightness(dev_ctx.level_control_attr.current_level);
			} else {
				light_set_brightness(0);
			}
			effect_type = ZB_ZCL_IDENTIFY_EFFECT_ID_STOP;
		}
		break;

	case ZB_ZCL_IDENTIFY_EFFECT_ID_FINISH_EFFECT:
	case ZB_ZCL_IDENTIFY_EFFECT_ID_STOP:
	default:
		/* Restore previous state */
		if (dev_ctx.on_off_attr.on_off) {
			light_set_brightness(dev_ctx.level_control_attr.current_level);
		} else {
			light_set_brightness(0);
		}
		effect_type = ZB_ZCL_IDENTIFY_EFFECT_ID_STOP;
		break;
	}
}

static void start_identify_effect(uint8_t effect_id)
{
	LOG_INF("Identify effect: %u", effect_id);

	/* Cancel any running effect */
	k_work_cancel_delayable(&effect_work);

	effect_type = effect_id;
	effect_step = 0;

	if (effect_id == ZB_ZCL_IDENTIFY_EFFECT_ID_STOP ||
	    effect_id == ZB_ZCL_IDENTIFY_EFFECT_ID_FINISH_EFFECT) {
		/* Immediately restore state */
		if (dev_ctx.on_off_attr.on_off) {
			light_set_brightness(dev_ctx.level_control_attr.current_level);
		} else {
			light_set_brightness(0);
		}
	} else {
		/* Start effect */
		k_work_schedule(&effect_work, K_NO_WAIT);
	}
}

/* ==========================================================================
 * Battery Measurement - LiPo via VDDH (nRF52840)
 * ========================================================================== */

/*
 * LiPo voltage to percentage lookup table.
 * Based on typical LiPo discharge curve with values in millivolts.
 * Converts VDDH voltage (from ADC) to battery percentage.
 *
 * LiPo characteristics:
 * - Full charge: 4.20V (100%)
 * - Nominal: 3.70V (~50%)
 * - Cutoff: 3.00V (0%) - below this risks damage
 *
 * The discharge curve is non-linear:
 * - Steep drop from 4.2V to ~4.0V
 * - Relatively flat from 4.0V to 3.6V
 * - Gradual drop from 3.6V to 3.3V
 * - Steep drop below 3.3V
 */
struct battery_level_point {
	uint16_t mv;      /* Voltage in millivolts */
	uint8_t  percent; /* Percentage (0-100) */
};

static const struct battery_level_point lipo_discharge_curve[] = {
	{ 4200, 100 },
	{ 4150,  95 },
	{ 4110,  90 },
	{ 4080,  85 },
	{ 4020,  80 },
	{ 3980,  75 },
	{ 3950,  70 },
	{ 3910,  65 },
	{ 3870,  60 },
	{ 3840,  55 },
	{ 3800,  50 },
	{ 3760,  45 },
	{ 3730,  40 },
	{ 3690,  35 },
	{ 3660,  30 },
	{ 3620,  25 },
	{ 3580,  20 },
	{ 3500,  15 },
	{ 3450,  10 },
	{ 3300,   5 },
	{ 3000,   0 },
};

#define BATTERY_CURVE_SIZE ARRAY_SIZE(lipo_discharge_curve)

/**
 * Convert battery voltage (mV) to percentage using lookup table with interpolation.
 */
static uint8_t battery_mv_to_percent(uint16_t mv)
{
	if (mv >= lipo_discharge_curve[0].mv) {
		return 100;
	}

	if (mv <= lipo_discharge_curve[BATTERY_CURVE_SIZE - 1].mv) {
		return 0;
	}

	/* Find the two points to interpolate between */
	for (size_t i = 0; i < BATTERY_CURVE_SIZE - 1; i++) {
		if (mv >= lipo_discharge_curve[i + 1].mv) {
			/* Linear interpolation */
			uint16_t v_high = lipo_discharge_curve[i].mv;
			uint16_t v_low = lipo_discharge_curve[i + 1].mv;
			uint8_t p_high = lipo_discharge_curve[i].percent;
			uint8_t p_low = lipo_discharge_curve[i + 1].percent;

			uint16_t v_range = v_high - v_low;
			uint8_t p_range = p_high - p_low;

			return p_low + ((mv - v_low) * p_range) / v_range;
		}
	}

	return 0;
}

/**
 * Measure VDDH voltage using nRF52840 SAADC.
 * Returns voltage in millivolts.
 */
static uint16_t battery_measure_mv(void)
{
	int16_t sample;
	uint16_t voltage_mv;

	if (!adc_dev) {
		LOG_ERR("ADC not initialized");
		return 0;
	}

	/* Configure SAADC for VDD measurement
	 * nRF52840 SAADC can measure VDD directly using internal channel
	 * Input: VDD/5 (internal divider)
	 * Reference: Internal 0.6V
	 * Gain: 1/6
	 * Resolution: 12-bit
	 *
	 * Voltage = (sample * reference * gain_divisor) / (resolution * input_divider)
	 * Voltage = (sample * 0.6 * 6) / (4096 * 1/5)
	 * Voltage = (sample * 3.6) / (4096 / 5)
	 * Voltage = (sample * 3.6 * 5) / 4096
	 * Voltage = sample * 18 / 4096 (in volts)
	 * Voltage_mV = sample * 18000 / 4096
	 */

	struct adc_channel_cfg channel_cfg = {
		.gain = ADC_GAIN_1_6,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = 0,
		.input_positive = SAADC_CH_PSELP_PSELP_VDD,
	};

	int ret = adc_channel_setup(adc_dev, &channel_cfg);
	if (ret < 0) {
		LOG_ERR("ADC channel setup failed: %d", ret);
		return 0;
	}

	struct adc_sequence sequence = {
		.channels = BIT(0),
		.buffer = &sample,
		.buffer_size = sizeof(sample),
		.resolution = 12,
	};

	ret = adc_read(adc_dev, &sequence);
	if (ret < 0) {
		LOG_ERR("ADC read failed: %d", ret);
		return 0;
	}

	/* Convert to millivolts
	 * VDD measurement uses internal 1/5 divider and 0.6V reference
	 * With gain 1/6: measured = VDD * (1/5) / (0.6 * 6) * 4096
	 * VDD = measured * 0.6 * 6 * 5 / 4096
	 * VDD_mV = measured * 18000 / 4096 = measured * 4.395
	 */
	voltage_mv = (uint32_t)sample * 18000U / 4096U;

	LOG_DBG("Battery ADC: %d -> %u mV", sample, voltage_mv);

	return voltage_mv;
}

/**
 * Update battery attributes and report to coordinator.
 */
static void battery_update_and_report(void)
{
	uint16_t voltage_mv = battery_measure_mv();

	if (voltage_mv == 0) {
		LOG_WRN("Battery measurement failed");
		return;
	}

	uint8_t percent = battery_mv_to_percent(voltage_mv);

	/* Update attributes
	 * battery_voltage is in units of 100mV (ZCL spec)
	 * battery_percentage is 0-200 (0.5% per unit, so 200 = 100%)
	 */
	dev_ctx.power_config_attr.battery_voltage = voltage_mv / 100;
	dev_ctx.power_config_attr.battery_percentage = percent * 2; /* Convert to 0.5% units */

	LOG_INF("Battery: %u mV (%u%%)", voltage_mv, percent);

	/* Report to coordinator if joined */
	if (ZB_JOINED()) {
		ZB_ZCL_SET_ATTRIBUTE(
			BATTERY_ENDPOINT,
			ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
			ZB_ZCL_CLUSTER_SERVER_ROLE,
			ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
			(zb_uint8_t *)&dev_ctx.power_config_attr.battery_voltage,
			ZB_FALSE);

		LOG_DBG("Battery level reported");
	}
}

/**
 * Battery report work handler - called periodically.
 */
static void battery_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	battery_update_and_report();

	/* Reschedule for next report */
	k_work_schedule(&battery_work, K_SECONDS(BATTERY_REPORT_INTERVAL_SEC));
}

/**
 * Initialize battery measurement.
 */
static int battery_init(void)
{
	adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	/* Initialize power config attributes */
	dev_ctx.power_config_attr.battery_voltage = 0;
	dev_ctx.power_config_attr.battery_percentage = 0;
	dev_ctx.power_config_attr.battery_size = ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_OTHER; /* LiPo */
	dev_ctx.power_config_attr.battery_quantity = 1;
	dev_ctx.power_config_attr.battery_rated_voltage = 37; /* 3.7V nominal in 100mV units */
	dev_ctx.power_config_attr.battery_alarm_mask = 0;
	dev_ctx.power_config_attr.battery_voltage_min_threshold = 30; /* 3.0V in 100mV units */

	/* Initialize work item */
	k_work_init_delayable(&battery_work, battery_work_handler);

	LOG_INF("Battery measurement initialized");

	return 0;
}

/**
 * Start periodic battery reporting.
 * Called after network join.
 */
static void battery_start_reporting(void)
{
	/* Do an immediate measurement and report */
	battery_update_and_report();

	/* Schedule periodic reports */
	k_work_schedule(&battery_work, K_SECONDS(BATTERY_REPORT_INTERVAL_SEC));

	LOG_INF("Battery reporting started (interval: %u sec)", BATTERY_REPORT_INTERVAL_SEC);
}

/* ==========================================================================
 * Status LED - Blinks when not joined, off when joined
 * ========================================================================== */

static struct k_work_delayable status_led_work;

static void status_led_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!device_is_ready(status_led.port)) {
		return;
	}

	if (ZB_JOINED()) {
		/* Joined - LED off, stop blinking */
		gpio_pin_set_dt(&status_led, 0);
	} else {
		/* Not joined - toggle LED and reschedule */
		gpio_pin_toggle_dt(&status_led);
		k_work_schedule(&status_led_work, K_MSEC(500));
	}
}

static void update_status_led(void)
{
	if (!device_is_ready(status_led.port)) {
		return;
	}

	if (ZB_JOINED()) {
		/* Joined - ensure LED is off and stop blinking */
		k_work_cancel_delayable(&status_led_work);
		gpio_pin_set_dt(&status_led, 0);
	} else {
		/* Not joined - start blinking if not already */
		if (!k_work_delayable_is_pending(&status_led_work)) {
			k_work_schedule(&status_led_work, K_NO_WAIT);
		}
	}
}

/* ==========================================================================
 * Button Handling
 * ========================================================================== */

static void button_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	bool pressed = (gpio_pin_get_dt(&button) == 1);

	if (pressed && !app_state.pressed) {
		/* Button pressed */
		app_state.pressed = true;
		app_state.press_time = k_uptime_get();
		k_work_schedule(&long_press_work, K_MSEC(BUTTON_LONG_PRESS_MS));
		LOG_DBG("Button pressed");
	} else if (!pressed && app_state.pressed) {
		/* Button released */
		app_state.pressed = false;
		k_work_cancel_delayable(&long_press_work);

		int64_t duration = k_uptime_get() - app_state.press_time;
		if (duration < BUTTON_LONG_PRESS_MS) {
			LOG_INF("Short press - toggle");
			light_toggle();
		}
		LOG_DBG("Button released after %lld ms", duration);
	}
}

static void long_press_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (app_state.pressed) {
		LOG_INF("Long press - factory reset");

		/* Leave network and restart steering */
		if (ZB_JOINED()) {
			zb_bdb_reset_via_local_action(0);
		}

		/* Blink LED to indicate reset */
		for (int i = 0; i < 6; i++) {
			gpio_pin_toggle_dt(&status_led);
			k_msleep(100);
		}
	}
}

static void button_gpio_handler(const struct device *dev,
				struct gpio_callback *cb,
				uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_work_submit(&button_work);
}

static int button_init(void)
{
	int ret;

	if (!device_is_ready(button.port)) {
		LOG_ERR("Button device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Button config failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Button interrupt config failed: %d", ret);
		return ret;
	}

	gpio_init_callback(&button_cb_data, button_gpio_handler, BIT(button.pin));
	ret = gpio_add_callback(button.port, &button_cb_data);
	if (ret < 0) {
		LOG_ERR("Button callback add failed: %d", ret);
		return ret;
	}

	k_work_init(&button_work, button_work_handler);
	k_work_init_delayable(&long_press_work, long_press_work_handler);

	LOG_INF("Button initialized on P0.%u", button.pin);
	return 0;
}

/* ==========================================================================
 * Zigbee Attribute Initialization
 * ========================================================================== */

static void clusters_attr_init(void)
{
	dev_ctx.basic_attr.zcl_version = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.app_version = BULB_INIT_BASIC_APP_VERSION;
	dev_ctx.basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
	dev_ctx.basic_attr.hw_version = BULB_INIT_BASIC_HW_VERSION;
	dev_ctx.basic_attr.power_source = ZB_ZCL_BASIC_POWER_SOURCE_BATTERY;
	dev_ctx.basic_attr.ph_env = BULB_INIT_BASIC_PH_ENV;

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.mf_name,
		BULB_INIT_BASIC_MANUF_NAME,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.model_id,
		BULB_INIT_BASIC_MODEL_ID,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.date_code,
		BULB_INIT_BASIC_DATE_CODE,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_DATE_CODE));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.location_id,
		BULB_INIT_BASIC_LOCATION_DESC,
		ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_LOCATION_DESC));

	dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	/* On/Off attributes */
	dev_ctx.on_off_attr.on_off = ZB_ZCL_ON_OFF_IS_OFF;
	dev_ctx.on_off_attr.global_scene_ctrl = ZB_TRUE;
	dev_ctx.on_off_attr.on_time = 0;
	dev_ctx.on_off_attr.off_wait_time = 0;
	dev_ctx.on_off_attr.start_up_on_off = ZB_ZCL_ON_OFF_STARTUP_PREVIOUS;

	/* Level Control attributes */
	dev_ctx.level_control_attr.current_level = ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE;
	dev_ctx.level_control_attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;
	dev_ctx.level_control_attr.options = 0;
	dev_ctx.level_control_attr.on_off_transition_time = 10; /* Default 1 second (in 1/10s units) */
	dev_ctx.level_control_attr.start_up_current_level = ZB_ZCL_LEVEL_STARTUP_PREVIOUS;

	ZB_ZCL_SET_ATTRIBUTE(
		LIGHT_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&dev_ctx.on_off_attr.on_off,
		ZB_FALSE);

	ZB_ZCL_SET_ATTRIBUTE(
		LIGHT_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
		(zb_uint8_t *)&dev_ctx.level_control_attr.current_level,
		ZB_FALSE);
}

/* ==========================================================================
 * Startup Behavior
 * ========================================================================== */

static void apply_startup_behavior(void)
{
	zb_bool_t on_off_state;
	zb_uint8_t level;

	LOG_INF("Applying startup behavior (on_off: 0x%02x, level: 0x%02x)",
		dev_ctx.on_off_attr.start_up_on_off,
		dev_ctx.level_control_attr.start_up_current_level);

	/* Determine startup level */
	switch (dev_ctx.level_control_attr.start_up_current_level) {
	case ZB_ZCL_LEVEL_STARTUP_MINIMUM:
		level = ZB_ZCL_LEVEL_CONTROL_LEVEL_MIN_VALUE;
		break;
	case ZB_ZCL_LEVEL_STARTUP_PREVIOUS:
		/* Keep current_level as-is (restored from NVS) */
		level = dev_ctx.level_control_attr.current_level;
		break;
	default:
		/* Specific level value */
		level = dev_ctx.level_control_attr.start_up_current_level;
		break;
	}

	/* Determine startup on/off state */
	switch (dev_ctx.on_off_attr.start_up_on_off) {
	case ZB_ZCL_ON_OFF_STARTUP_OFF:
		on_off_state = ZB_FALSE;
		break;
	case ZB_ZCL_ON_OFF_STARTUP_ON:
		on_off_state = ZB_TRUE;
		break;
	case ZB_ZCL_ON_OFF_STARTUP_TOGGLE:
		on_off_state = !dev_ctx.on_off_attr.on_off;
		break;
	case ZB_ZCL_ON_OFF_STARTUP_PREVIOUS:
	default:
		/* Keep on_off as-is (restored from NVS) */
		on_off_state = dev_ctx.on_off_attr.on_off;
		break;
	}

	/* Apply the startup state */
	dev_ctx.level_control_attr.current_level = level;
	dev_ctx.on_off_attr.on_off = on_off_state;

	if (on_off_state) {
		light_set_brightness(level);
		app_state.last_brightness = level;
	} else {
		light_set_brightness(0);
	}

	LOG_INF("Startup state: %s, level: %u",
		on_off_state ? "ON" : "OFF", level);
}

/* ==========================================================================
 * Zigbee FOTA (Over-The-Air Updates)
 * ========================================================================== */

#ifdef CONFIG_ZIGBEE_FOTA
static void fota_evt_handler(const struct zigbee_fota_evt *evt)
{
	switch (evt->id) {
	case ZIGBEE_FOTA_EVT_PROGRESS:
		LOG_INF("OTA progress: %d%%", evt->dl.progress);
		/* Blink status LED during download */
		if (device_is_ready(status_led.port)) {
			gpio_pin_toggle_dt(&status_led);
		}
		break;

	case ZIGBEE_FOTA_EVT_FINISHED:
		LOG_INF("OTA download complete, rebooting...");
		sys_reboot(SYS_REBOOT_COLD);
		break;

	case ZIGBEE_FOTA_EVT_ERROR:
		LOG_ERR("OTA transfer failed");
		break;

	default:
		break;
	}
}
#endif

/* ==========================================================================
 * Zigbee Callbacks
 * ========================================================================== */

static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_zcl_device_callback_param_t *param =
		ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

	param->status = RET_OK;

	switch (param->device_cb_id) {
	case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
		level_control_set_value(
			param->cb_param.level_control_set_value_param.new_value);
		break;

	case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
		if (param->cb_param.set_attr_value_param.cluster_id ==
		    ZB_ZCL_CLUSTER_ID_ON_OFF) {
			on_off_set_value(
				(zb_bool_t)param->cb_param.set_attr_value_param.values.data8);
		} else if (param->cb_param.set_attr_value_param.cluster_id ==
			   ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
			level_control_set_value(
				param->cb_param.set_attr_value_param.values.data16);
		} else {
			param->status = RET_NOT_IMPLEMENTED;
		}
		break;

	case ZB_ZCL_IDENTIFY_EFFECT_CB_ID:
		start_identify_effect(
			param->cb_param.identify_effect_value_param.effect_id);
		break;

#ifdef CONFIG_ZIGBEE_FOTA
	case ZB_ZCL_OTA_UPGRADE_VALUE_CB_ID:
		zigbee_fota_zcl_cb(bufid);
		break;
#endif

	default:
		param->status = RET_NOT_IMPLEMENTED;
		break;
	}
}

/* Sleepy End Device poll interval in ms (how often to check for messages) */
#define SED_POLL_INTERVAL_MS 3000

void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t *sig_hdr = NULL;
	zb_zdo_app_signal_type_t sig_type = zb_get_app_signal(bufid, &sig_hdr);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);

	/* Update status LED */
	update_status_led();

#ifdef CONFIG_ZIGBEE_FOTA
	/* Pass signals to FOTA library */
	zigbee_fota_signal_handler(bufid);
#endif

	/* Configure sleepy device after successful join/rejoin */
	if (sig_type == ZB_BDB_SIGNAL_DEVICE_FIRST_START ||
	    sig_type == ZB_BDB_SIGNAL_DEVICE_REBOOT) {
		if (status == RET_OK) {
			/* Set poll interval for sleepy end device */
			zb_zdo_pim_set_long_poll_interval(SED_POLL_INTERVAL_MS);
			LOG_INF("Sleepy End Device: poll interval %d ms", SED_POLL_INTERVAL_MS);

			/* Start battery reporting now that we've joined */
			battery_start_reporting();
		}
	}

	/* Use default signal handler */
	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	if (bufid) {
		zb_buf_free(bufid);
	}
}

/* ==========================================================================
 * Initialization
 * ========================================================================== */

static int hardware_init(void)
{
	int ret;

	/* PWM */
	if (!device_is_ready(pwm_brightness.dev)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}
	LOG_INF("PWM ready: period=%u ns", pwm_brightness.period);

	/* TB6612 H-Bridge */
	ret = tb6612_init();
	if (ret < 0) {
		return ret;
	}

	/* Status LED */
	if (device_is_ready(status_led.port)) {
		ret = gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_WRN("Status LED config failed: %d", ret);
		}
	}

	/* Button */
	ret = button_init();
	if (ret < 0) {
		return ret;
	}

	/* Battery measurement */
	ret = battery_init();
	if (ret < 0) {
		LOG_WRN("Battery init failed: %d (continuing without battery)", ret);
		/* Don't fail - battery is optional */
	}

	/* Initialize work items */
	k_work_init_delayable(&effect_work, effect_work_handler);
	k_work_init_delayable(&status_led_work, status_led_work_handler);
	k_work_init_delayable(&transition_work, transition_work_handler);

	/* Start with light off */
	light_set_brightness(0);

	return 0;
}

/* ==========================================================================
 * Main
 * ========================================================================== */

int main(void)
{
	int err;

	LOG_INF("========================================");
	LOG_INF("LED Copper String Controller v1.0.0");
	LOG_INF("Board: %s", CONFIG_BOARD);
	LOG_INF("TB6612 Polarity: %u Hz", 1000000U / POLARITY_PERIOD_US);
	LOG_INF("========================================");

	err = hardware_init();
	if (err) {
		LOG_ERR("Hardware init failed: %d", err);
		return err;
	}

	err = settings_subsys_init();
	if (err) {
		LOG_ERR("Settings init failed: %d", err);
	}

#ifdef CONFIG_ZIGBEE_FOTA
	/* Initialize OTA client */
	err = zigbee_fota_init(fota_evt_handler);
	if (err) {
		LOG_ERR("FOTA init failed: %d", err);
	}

	/* Confirm current image to prevent rollback on next boot */
	if (!boot_is_img_confirmed()) {
		err = boot_write_img_confirmed();
		if (err) {
			LOG_ERR("Failed to confirm image: %d", err);
		} else {
			LOG_INF("Image confirmed");
		}
	}
#endif

	/* Register ZCL device callback */
	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

	/* Register device context */
	ZB_AF_REGISTER_DEVICE_CTX(&light_ctx);

	/* Initialize cluster attributes */
	clusters_attr_init();

	/* Load settings (restores previous on/off and level states) */
	err = settings_load();
	if (err) {
		LOG_ERR("Settings load failed: %d", err);
	}

	/* Apply startup behavior based on configuration */
	apply_startup_behavior();

	LOG_INF("Hold button 3s to reset/pair");
	LOG_INF("Starting Zigbee stack...");

	/* Enable sleepy end device (radio off between polls)
	 * ED_AGING_TIMEOUT_64MIN = parent keeps us in table for 64 min without contact
	 * This allows deep sleep while still being reachable for commands
	 */
	zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
	zigbee_configure_sleepy_behavior(true);

	/* Start Zigbee stack */
	zigbee_enable();

	/* Main loop - keep thread alive */
	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
