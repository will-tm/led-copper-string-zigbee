/**
 * @file main.c
 * @brief LED Copper String Zigbee Controller
 *
 * Controls a special LED strip via TB6612 H-bridge where:
 * - One polarity lights half the strip
 * - Reverse polarity lights the other half
 *
 * Both polarities are generated in hardware by one PWM instance:
 * - AIN1 (PWM ch0, normal):   high during [0, D) of every period
 * - AIN2 (PWM ch1, inverted): high during [T-D, T) of every period
 * With D <= T/2 the two windows never overlap, so each half of the strip is
 * driven for D per period and brightness is set by D alone. PWMA and STBY are
 * plain GPIOs held high while the output is non-zero. No CPU activity is
 * needed to keep the light on, and at 0 the PWM peripheral is stopped.
 *
 * Threading:
 * - All TB6612/PWM output changes run on the system workqueue.
 * - All ZBOSS API calls run in the Zigbee thread; other contexts hand work
 *   over with ZB_SCHEDULE_APP_CALLBACK (thread-safe in NCS).
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_saadc.h>
#include <hal/nrf_power.h>
#include <nrfx_power.h>
#include <ram_pwrdn.h>

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

/* Valid CurrentLevel range for lighting devices (ZCL 3.10.2.3). */
#define LEVEL_MIN                       1U
#define LEVEL_MAX                       254U

/* Startup behavior values for On/Off cluster */
#define ZB_ZCL_ON_OFF_STARTUP_OFF       0x00
#define ZB_ZCL_ON_OFF_STARTUP_ON        0x01
#define ZB_ZCL_ON_OFF_STARTUP_TOGGLE    0x02
#define ZB_ZCL_ON_OFF_STARTUP_PREVIOUS  0xFF

/* Startup behavior values for Level Control cluster */
#define ZB_ZCL_LEVEL_STARTUP_MINIMUM    0x00
#define ZB_ZCL_LEVEL_STARTUP_PREVIOUS   0xFF

/* Default OnOffTransitionTime, 1/10 s */
#define ON_OFF_TRANSITION_DEFAULT       10U

/* Fade update period while a transition runs */
#define FADE_STEP_MS                    16

/*
 * ZBOSS runs Level Control transitions itself, one level per callback.
 * Each step is faded over the time since the previous step so 8-bit steps
 * are not visible; gaps longer than this are treated as a new jump.
 */
#define LEVEL_STEP_SMOOTH_MAX_MS        250

/* Button */
#define BUTTON_DEBOUNCE_MS              30
#define BUTTON_LONG_PRESS_MS            3000

/* Status LED blinks while not joined, for at most this long per trigger. */
#define JOIN_BLINK_TIMEOUT_MS           (3 * 60 * 1000)
#define JOIN_BLINK_PERIOD_MS            500

/*
 * Sleepy End Device poll intervals. The output stage draws far more than
 * the radio while lit, so poll fast then for responsive control; poll slowly
 * while off, where battery life is decided.
 */
#define POLL_INTERVAL_FAST_MS           250
#define POLL_INTERVAL_SLOW_MS           3000
/* Fast polling after joining so the coordinator can interview the device. */
#define INTERVIEW_FAST_POLL_MS          (2 * 60 * 1000)

/* Delay before persisting state, so bursts of changes cost one flash write. */
#define STATE_SAVE_DELAY_MS             2000

/*
 * Watchdog: reset if the Zigbee thread or the system workqueue stops running.
 * The workqueue feeds only after the Zigbee thread answered its last ping.
 */
#define WDT_TIMEOUT_MS                  60000
#define WDT_FEED_INTERVAL_MS            15000

/* USB power must be present this long before rebooting into the bootloader */
#define USB_REBOOT_DELAY_MS             500

/* Battery measurement interval */
#define BATTERY_REPORT_INTERVAL_SEC     CONFIG_APP_BATTERY_REPORT_INTERVAL_SEC

/*
 * Low-battery protection on the resting voltage: the output is forced off
 * below CUTOFF and allowed again from RESUME. Released while USB is present.
 */
#define BATTERY_CUTOFF_MV               3300
#define BATTERY_RESUME_MV               3500

/* Let the cell relax after charging before measuring it */
#define BATTERY_AFTER_USB_DELAY_SEC     30

/* Fresh battery report this long after (re)joining the network */
#define BATTERY_JOIN_REPORT_DELAY_SEC   10

/* ==========================================================================
 * Device Tree
 * ========================================================================== */

/* TB6612 AIN1/AIN2 driven by the same PWM instance (shared period) */
static const struct pwm_dt_spec pwm_ain1 = PWM_DT_SPEC_GET(DT_NODELABEL(tb6612_ain1));
static const struct pwm_dt_spec pwm_ain2 = PWM_DT_SPEC_GET(DT_NODELABEL(tb6612_ain2));

BUILD_ASSERT(DT_SAME_NODE(DT_PWMS_CTLR(DT_NODELABEL(tb6612_ain1)),
			  DT_PWMS_CTLR(DT_NODELABEL(tb6612_ain2))),
	     "AIN1 and AIN2 must share one PWM instance");
BUILD_ASSERT(DT_PWMS_PERIOD(DT_NODELABEL(tb6612_ain1)) ==
	     DT_PWMS_PERIOD(DT_NODELABEL(tb6612_ain2)),
	     "AIN1 and AIN2 must use the same PWM period");
BUILD_ASSERT((DT_PWMS_FLAGS(DT_NODELABEL(tb6612_ain2)) & PWM_POLARITY_INVERTED) &&
	     !(DT_PWMS_FLAGS(DT_NODELABEL(tb6612_ain1)) & PWM_POLARITY_INVERTED),
	     "AIN1 must be normal and AIN2 inverted polarity");

/* TB6612 static control pins */
static const struct gpio_dt_spec tb6612_pwma = GPIO_DT_SPEC_GET(DT_NODELABEL(tb6612_pwma), gpios);
static const struct gpio_dt_spec tb6612_standby = GPIO_DT_SPEC_GET(DT_NODELABEL(tb6612_standby), gpios);

/* Button and status LED */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* Battery voltage divider control (P0.29 - drive LOW to enable, high-Z to disable) */
static const struct gpio_dt_spec batt_divider_gnd = GPIO_DT_SPEC_GET(DT_NODELABEL(batt_divider_gnd), gpios);

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
	zb_uint8_t  battery_quantity;
	zb_uint8_t  battery_rated_voltage;    /* In units of 100mV */
	zb_uint8_t  battery_alarm_mask;
	zb_uint8_t  battery_voltage_min_threshold;
	zb_uint16_t battery_voltage_raw_mv;   /* Raw voltage in mV (manufacturer-specific) */
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

/* Manufacturer-specific attribute ID for raw battery voltage in mV */
#define ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_RAW_MV_ID  0xFF00

/* Power Configuration cluster attribute list for battery (custom to include percentage and raw mV) */
ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(power_config_attr_list, ZB_ZCL_POWER_CONFIG)
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID(&dev_ctx.power_config_attr.battery_voltage, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID(&dev_ctx.power_config_attr.battery_percentage, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID(&dev_ctx.power_config_attr.battery_size, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID(&dev_ctx.power_config_attr.battery_quantity, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_RATED_VOLTAGE_ID(&dev_ctx.power_config_attr.battery_rated_voltage, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_MASK_ID(&dev_ctx.power_config_attr.battery_alarm_mask, ),
ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_MIN_THRESHOLD_ID(&dev_ctx.power_config_attr.battery_voltage_min_threshold, ),
/* Manufacturer-specific: raw voltage in mV (uint16) */
{
  ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_RAW_MV_ID,
  ZB_ZCL_ATTR_TYPE_U16,
  ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING,
  (ZB_ZCL_NON_MANUFACTURER_SPECIFIC),
  (void*) &dev_ctx.power_config_attr.battery_voltage_raw_mv
},
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

/* Reporting contexts - include Power Configuration for battery reporting */
#define POWER_CONFIG_REPORT_ATTR_COUNT 3  /* battery_voltage, battery_percentage, raw_mv */
#define LIGHT_REPORT_ATTR_COUNT (ZB_ZCL_ON_OFF_REPORT_ATTR_COUNT + ZB_ZCL_LEVEL_CONTROL_REPORT_ATTR_COUNT + POWER_CONFIG_REPORT_ATTR_COUNT)
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
 * TB6612 Output (system workqueue only)
 * ========================================================================== */

static uint32_t pwm_period_cycles;
static bool output_enabled;

/* Set while the battery is below the cutoff: output is held off. */
static atomic_t battery_low;

/*
 * CIE 1931 lightness correction.
 * Maps perceptual lightness (0-65535 = L* 0-100) to a pulse in 0..max.
 *   Y = L* / 903.3              for L* <= 8
 *   Y = ((L* + 16) / 116)^3     for L* > 8
 */
#define CIE_LINEAR_LIMIT  5242U  /* L* = 8 */

static uint32_t cie1931_correct(uint16_t lightness, uint32_t max)
{
	uint64_t y;

	if (lightness == 0) {
		return 0;
	}

	if (lightness <= CIE_LINEAR_LIMIT) {
		/* Y * max = (100 * l / 65535) / 903.3 * max */
		y = (uint64_t)lightness * max * 1000U / (65535ULL * 9033U);
	} else {
		/* u = (L* + 16) / 116 in Q16, at most 65536 */
		uint64_t u = ((uint64_t)lightness * 100U + 16ULL * 65535U) * 65536U /
			     (116ULL * 65535U);

		y = (((u * u * u) >> 16) * max) >> 32;
	}

	/* Keep the lowest non-zero level visible */
	return (y == 0) ? 1 : (uint32_t)MIN(y, max);
}

static int output_init(void)
{
	uint64_t cycles_per_sec;
	int ret;

	if (!pwm_is_ready_dt(&pwm_ain1) || !pwm_is_ready_dt(&pwm_ain2)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}

	ret = pwm_get_cycles_per_sec(pwm_ain1.dev, pwm_ain1.channel, &cycles_per_sec);
	if (ret < 0) {
		return ret;
	}
	pwm_period_cycles = (uint32_t)(cycles_per_sec * pwm_ain1.period / NSEC_PER_SEC);

	if (!gpio_is_ready_dt(&tb6612_pwma) || !gpio_is_ready_dt(&tb6612_standby)) {
		LOG_ERR("TB6612 GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&tb6612_pwma, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&tb6612_standby, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("TB6612 PWM period %u cycles", pwm_period_cycles);
	return 0;
}

/* Drive the strip at the given perceptual lightness (0 = off, standby). */
static void output_apply(uint16_t lightness)
{
	/* Each polarity may be driven for at most half of the period. */
	uint32_t pulse = atomic_get(&battery_low) ? 0 :
			 cie1931_correct(lightness, pwm_period_cycles / 2);

	if (pulse > 0 && !output_enabled) {
		gpio_pin_set_dt(&tb6612_standby, 1);
		gpio_pin_set_dt(&tb6612_pwma, 1);
		output_enabled = true;
	}

	/* AIN2 is inverted: low for (period - pulse), then high for pulse. */
	if (pwm_set_cycles(pwm_ain1.dev, pwm_ain1.channel, pwm_period_cycles,
			   pulse, pwm_ain1.flags) ||
	    pwm_set_cycles(pwm_ain2.dev, pwm_ain2.channel, pwm_period_cycles,
			   pwm_period_cycles - pulse, pwm_ain2.flags)) {
		LOG_ERR("PWM set failed");
	}

	if (pulse == 0 && output_enabled) {
		gpio_pin_set_dt(&tb6612_pwma, 0);
		gpio_pin_set_dt(&tb6612_standby, 0);
		output_enabled = false;
	}
}

/* ==========================================================================
 * Fades (output updated on the system workqueue)
 * ========================================================================== */

static struct k_work_delayable fade_work;
static struct k_spinlock fade_lock;
static struct {
	uint16_t from;
	uint16_t to;
	uint16_t now;
	int64_t  start;
	uint32_t duration_ms;
} fade;

static inline uint16_t level_to_lightness(uint8_t level)
{
	level = MIN(level, LEVEL_MAX);
	return (uint16_t)((uint32_t)level * UINT16_MAX / LEVEL_MAX);
}

static void fade_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_spinlock_key_t key = k_spin_lock(&fade_lock);
	int64_t elapsed = k_uptime_get() - fade.start;
	bool done = (elapsed >= (int64_t)fade.duration_ms);
	uint16_t value;

	if (done) {
		value = fade.to;
	} else {
		int64_t diff = (int64_t)fade.to - (int64_t)fade.from;

		value = (uint16_t)(fade.from + diff * elapsed / (int64_t)fade.duration_ms);
	}
	fade.now = value;
	k_spin_unlock(&fade_lock, key);

	output_apply(value);

	if (!done) {
		k_work_schedule(&fade_work, K_MSEC(FADE_STEP_MS));
	}
}

/* Fade from the current output to target. Callable from any thread. */
static void fade_to(uint16_t target, uint32_t duration_ms)
{
	k_spinlock_key_t key = k_spin_lock(&fade_lock);

	fade.from = fade.now;
	fade.to = target;
	fade.start = k_uptime_get();
	fade.duration_ms = duration_ms;
	k_spin_unlock(&fade_lock, key);

	k_work_reschedule(&fade_work, K_NO_WAIT);
}

static uint16_t fade_now(void)
{
	k_spinlock_key_t key = k_spin_lock(&fade_lock);
	uint16_t now = fade.now;

	k_spin_unlock(&fade_lock, key);
	return now;
}

/* ==========================================================================
 * Light State
 * ========================================================================== */

/* True while an Identify effect owns the output. */
static atomic_t effect_active;

static uint16_t state_target_lightness(void)
{
	uint8_t level = CLAMP(dev_ctx.level_control_attr.current_level, LEVEL_MIN, LEVEL_MAX);

	return dev_ctx.on_off_attr.on_off ? level_to_lightness(level) : 0;
}

/* Move the output to the Zigbee On/Off + Level state, unless an effect runs. */
static void light_sync(uint32_t duration_ms)
{
	if (!atomic_get(&effect_active)) {
		fade_to(state_target_lightness(), duration_ms);
	}
}

static uint32_t on_off_transition_ms(void)
{
	return (uint32_t)dev_ctx.level_control_attr.on_off_transition_time * 100U;
}

/* --------------------------------------------------------------------------
 * Persistence: save on/off, level and the writable configuration attributes.
 * -------------------------------------------------------------------------- */

static struct k_work_delayable save_work;

static int light_settings_set(const char *name, size_t len,
			      settings_read_cb read_cb, void *cb_arg)
{
	void *dst;
	size_t size;

	if (!strcmp(name, "on_off")) {
		dst = &dev_ctx.on_off_attr.on_off;
		size = sizeof(dev_ctx.on_off_attr.on_off);
	} else if (!strcmp(name, "level")) {
		dst = &dev_ctx.level_control_attr.current_level;
		size = sizeof(dev_ctx.level_control_attr.current_level);
	} else if (!strcmp(name, "su_on_off")) {
		dst = &dev_ctx.on_off_attr.start_up_on_off;
		size = sizeof(dev_ctx.on_off_attr.start_up_on_off);
	} else if (!strcmp(name, "su_level")) {
		dst = &dev_ctx.level_control_attr.start_up_current_level;
		size = sizeof(dev_ctx.level_control_attr.start_up_current_level);
	} else if (!strcmp(name, "transition")) {
		dst = &dev_ctx.level_control_attr.on_off_transition_time;
		size = sizeof(dev_ctx.level_control_attr.on_off_transition_time);
	} else {
		return -ENOENT;
	}

	if (len != size) {
		return -EINVAL;
	}

	return (read_cb(cb_arg, dst, len) == (ssize_t)len) ? 0 : -EIO;
}

SETTINGS_STATIC_HANDLER_DEFINE(light, "light", NULL, light_settings_set, NULL, NULL);

static void save_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* NVS skips the write when a value is unchanged. */
	settings_save_one("light/on_off", &dev_ctx.on_off_attr.on_off,
			  sizeof(dev_ctx.on_off_attr.on_off));
	settings_save_one("light/level", &dev_ctx.level_control_attr.current_level,
			  sizeof(dev_ctx.level_control_attr.current_level));
	settings_save_one("light/su_on_off", &dev_ctx.on_off_attr.start_up_on_off,
			  sizeof(dev_ctx.on_off_attr.start_up_on_off));
	settings_save_one("light/su_level", &dev_ctx.level_control_attr.start_up_current_level,
			  sizeof(dev_ctx.level_control_attr.start_up_current_level));
	settings_save_one("light/transition", &dev_ctx.level_control_attr.on_off_transition_time,
			  sizeof(dev_ctx.level_control_attr.on_off_transition_time));
}

static void state_save_schedule(void)
{
	k_work_reschedule(&save_work, K_MSEC(STATE_SAVE_DELAY_MS));
}

/* ==========================================================================
 * Sleepy End Device poll control (Zigbee thread only)
 * ========================================================================== */

static bool interview_window;
static bool ota_active;
static zb_time_t poll_interval_ms;

static void poll_update(void)
{
	zb_time_t interval;

	if (!ZB_JOINED()) {
		poll_interval_ms = 0;
		return;
	}

	interval = (dev_ctx.on_off_attr.on_off || interview_window || ota_active) ?
		   POLL_INTERVAL_FAST_MS : POLL_INTERVAL_SLOW_MS;

	if (interval != poll_interval_ms) {
		zb_zdo_pim_set_long_poll_interval(interval);
		poll_interval_ms = interval;
		LOG_INF("Poll interval %u ms", (unsigned int)interval);
	}
}

static void interview_window_end(zb_uint8_t param)
{
	ARG_UNUSED(param);

	interview_window = false;
	poll_update();
}

/* ==========================================================================
 * Zigbee-side light control (Zigbee thread only)
 * ========================================================================== */

static int64_t last_level_step_time;
static void battery_low_force_off(zb_uint8_t param);

static void level_control_set_value(zb_uint8_t new_level)
{
	int64_t now = k_uptime_get();
	int64_t since_last = now - last_level_step_time;

	last_level_step_time = now;
	new_level = CLAMP(new_level, LEVEL_MIN, LEVEL_MAX);

	LOG_DBG("Set level: %u", new_level);

	ZB_ZCL_SET_ATTRIBUTE(
		LIGHT_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
		&new_level,
		ZB_FALSE);

	light_sync((since_last <= LEVEL_STEP_SMOOTH_MAX_MS) ? (uint32_t)since_last : 0);
	poll_update();
	state_save_schedule();
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

	light_sync(on_off_transition_ms());
	poll_update();
	state_save_schedule();

	if (on && atomic_get(&battery_low)) {
		LOG_WRN("Battery low: staying off");
		ZB_SCHEDULE_APP_CALLBACK(battery_low_force_off, 0);
	}
}

/* ==========================================================================
 * Level Control commands handled by the application (Zigbee thread)
 *
 * ZBOSS runs level transitions in coarse steps. Move to Level, Step and their
 * "with On/Off" variants are handled here so a transition is one continuous
 * fade over exactly the requested time. Move (rate based) and Stop are left
 * to ZBOSS; Stop also halts a fade started here.
 * ========================================================================== */

static uint32_t level_transition_ms(zb_uint16_t transition_time)
{
	return (transition_time == ZB_ZCL_LEVEL_CONTROL_TRANSITION_TIME_AS_FAST_AS_ABLE) ?
	       0 : (uint32_t)transition_time * 100U;
}

static zb_uint8_t lightness_to_level(uint16_t lightness)
{
	return (zb_uint8_t)(((uint32_t)lightness * LEVEL_MAX + UINT16_MAX / 2) / UINT16_MAX);
}

/* Update the On/Off attribute without driving the output. */
static void on_off_attr_set(zb_bool_t on)
{
	if (dev_ctx.on_off_attr.on_off == on) {
		return;
	}

	ZB_ZCL_SET_ATTRIBUTE(LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_ON_OFF,
			     ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			     (zb_uint8_t *)&on, ZB_FALSE);
	poll_update();

	if (on && atomic_get(&battery_low)) {
		LOG_WRN("Battery low: staying off");
		ZB_SCHEDULE_APP_CALLBACK(battery_low_force_off, 0);
	}
}

/* ExecuteIfOff after applying the command's OptionsMask/OptionsOverride. */
static bool level_execute_if_off(const zb_uint8_t *opts, zb_uint32_t len)
{
	zb_uint8_t options = dev_ctx.level_control_attr.options;

	if (len >= 2) {
		options = (options & ~opts[0]) | (opts[1] & opts[0]);
	}
	return options & BIT(ZB_ZCL_LEVEL_CONTROL_OPTIONS_EXECUTE_IF_OFF);
}

/*
 * Move to the target level over duration_ms. With On/Off, a level below
 * LEVEL_MIN fades out and switches off while CurrentLevel keeps the previous
 * level, so the next On restores it.
 */
static void level_move_to(int target, uint32_t duration_ms, bool with_on_off,
			  bool execute_if_off)
{
	if (with_on_off) {
		if (target < (int)LEVEL_MIN) {
			on_off_attr_set(ZB_FALSE);
			light_sync(duration_ms);
			state_save_schedule();
			return;
		}
		on_off_attr_set(ZB_TRUE);
	} else if (!dev_ctx.on_off_attr.on_off && !execute_if_off) {
		return;
	}

	zb_uint8_t level = CLAMP(target, LEVEL_MIN, LEVEL_MAX);

	ZB_ZCL_SET_ATTRIBUTE(LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
			     ZB_ZCL_CLUSTER_SERVER_ROLE,
			     ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &level, ZB_FALSE);
	light_sync(duration_ms);
	state_save_schedule();

	LOG_INF("Level -> %u over %u ms%s", level, duration_ms, with_on_off ? " (on/off)" : "");
}

static zb_uint8_t light_ep_handler(zb_bufid_t bufid)
{
	zb_zcl_parsed_hdr_t *cmd = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);
	const zb_uint8_t *p = zb_buf_begin(bufid);
	zb_uint32_t len = zb_buf_len(bufid);
	zb_uint8_t status = ZB_ZCL_STATUS_SUCCESS;
	bool with_on_off;

	if (cmd->cluster_id != ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL || cmd->is_common_command ||
	    cmd->is_manuf_specific || cmd->cmd_direction != ZB_ZCL_FRAME_DIRECTION_TO_SRV) {
		return ZB_FALSE;
	}

	switch (cmd->cmd_id) {
	case ZB_ZCL_CMD_LEVEL_CONTROL_MOVE_TO_LEVEL:
	case ZB_ZCL_CMD_LEVEL_CONTROL_MOVE_TO_LEVEL_WITH_ON_OFF:
		/* Level (u8), TransitionTime (u16) [, OptionsMask, OptionsOverride] */
		if (len < 3) {
			status = ZB_ZCL_STATUS_MALFORMED_CMD;
			break;
		}
		with_on_off = (cmd->cmd_id == ZB_ZCL_CMD_LEVEL_CONTROL_MOVE_TO_LEVEL_WITH_ON_OFF);
		level_move_to(p[0], level_transition_ms(p[1] | (p[2] << 8)), with_on_off,
			      level_execute_if_off(p + 3, len - 3));
		break;

	case ZB_ZCL_CMD_LEVEL_CONTROL_STEP:
	case ZB_ZCL_CMD_LEVEL_CONTROL_STEP_WITH_ON_OFF: {
		/* StepMode (0 up, 1 down), StepSize (u8), TransitionTime (u16) [, options] */
		if (len < 4 || p[0] > 1) {
			status = (len < 4) ? ZB_ZCL_STATUS_MALFORMED_CMD : ZB_ZCL_STATUS_INVALID_FIELD;
			break;
		}
		with_on_off = (cmd->cmd_id == ZB_ZCL_CMD_LEVEL_CONTROL_STEP_WITH_ON_OFF);

		int base = (with_on_off && !dev_ctx.on_off_attr.on_off) ?
			   (int)LEVEL_MIN : (int)dev_ctx.level_control_attr.current_level;
		int target = (p[0] == 0) ? base + p[1] : base - p[1];

		if (p[0] == 0 && target > (int)LEVEL_MAX) {
			target = LEVEL_MAX;
		}
		level_move_to(target, level_transition_ms(p[2] | (p[3] << 8)), with_on_off,
			      level_execute_if_off(p + 4, len - 4));
		break;
	}

	case ZB_ZCL_CMD_LEVEL_CONTROL_STOP:
	case ZB_ZCL_CMD_LEVEL_CONTROL_STOP_WITH_ON_OFF:
		/* Freeze a fade started here; ZBOSS then stops its own Move. */
		if (!atomic_get(&effect_active) && dev_ctx.on_off_attr.on_off) {
			uint16_t now = fade_now();
			zb_uint8_t level = CLAMP(lightness_to_level(now), LEVEL_MIN, LEVEL_MAX);

			fade_to(now, 0);
			ZB_ZCL_SET_ATTRIBUTE(LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
					     ZB_ZCL_CLUSTER_SERVER_ROLE,
					     ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
					     &level, ZB_FALSE);
			state_save_schedule();
		}
		return ZB_FALSE;

	default:
		return ZB_FALSE;
	}

	ZB_ZCL_PROCESS_COMMAND_FINISH(bufid, cmd, status);
	return ZB_TRUE;
}

/* ==========================================================================
 * Identify Effects (system workqueue)
 * ========================================================================== */

/* Internal effect id for the Identify command (blink until identify_time ends) */
#define EFFECT_ID_IDENTIFY      0x80
#define EFFECT_NO_REQUEST       (-1)

#define LIGHTNESS_FULL          UINT16_MAX
#define LIGHTNESS_DIM           level_to_lightness(25)

static struct k_work_delayable effect_work;
static atomic_t effect_request = ATOMIC_INIT(EFFECT_NO_REQUEST);
static uint8_t effect_id;
static uint16_t effect_step;

/* Run one step of the current effect; returns delay to next step, 0 when done. */
static uint32_t effect_step_run(uint16_t step)
{
	switch (effect_id) {
	case ZB_ZCL_IDENTIFY_EFFECT_ID_BLINK:
		if (step == 0) {
			fade_to(LIGHTNESS_FULL, 0);
			return 500;
		}
		return 0;

	case ZB_ZCL_IDENTIFY_EFFECT_ID_BREATHE:
		/* 15 smooth on/off cycles over 15 s */
		if (step < 30) {
			fade_to((step % 2 == 0) ? LIGHTNESS_FULL : 0, 500);
			return 500;
		}
		return 0;

	case ZB_ZCL_IDENTIFY_EFFECT_ID_OKAY:
		/* Two quick flashes */
		if (step < 4) {
			fade_to((step % 2 == 0) ? LIGHTNESS_FULL : 0, 0);
			return 200;
		}
		return 0;

	case ZB_ZCL_IDENTIFY_EFFECT_ID_CHANNEL_CHANGE:
		/* Full for 0.5 s, then dim for 7.5 s */
		if (step == 0) {
			fade_to(LIGHTNESS_FULL, 0);
			return 500;
		} else if (step == 1) {
			fade_to(LIGHTNESS_DIM, 0);
			return 7500;
		}
		return 0;

	case EFFECT_ID_IDENTIFY:
		/* Blink at 1 Hz until stopped */
		fade_to((step % 2 == 0) ? LIGHTNESS_FULL : 0, 0);
		return 500;

	default:
		/* STOP, FINISH_EFFECT and unknown ids end the effect */
		return 0;
	}
}

static void effect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	atomic_val_t req = atomic_set(&effect_request, EFFECT_NO_REQUEST);
	uint32_t next_ms;

	if (req != EFFECT_NO_REQUEST) {
		effect_id = (uint8_t)req;
		effect_step = 0;
	}

	next_ms = effect_step_run(effect_step);
	effect_step++;

	if (next_ms > 0) {
		k_work_schedule(&effect_work, K_MSEC(next_ms));
	} else if (atomic_get(&effect_request) == EFFECT_NO_REQUEST) {
		atomic_clear(&effect_active);
		fade_to(state_target_lightness(), 0);
	}
}

/* Start (or stop, for STOP/FINISH) an effect. Callable from any thread. */
static void effect_start(uint8_t id)
{
	LOG_INF("Identify effect: 0x%02x", id);

	if (id != ZB_ZCL_IDENTIFY_EFFECT_ID_STOP &&
	    id != ZB_ZCL_IDENTIFY_EFFECT_ID_FINISH_EFFECT) {
		atomic_set(&effect_active, 1);
	}
	atomic_set(&effect_request, id);
	k_work_reschedule(&effect_work, K_NO_WAIT);
}

/* Identify command notification: bufid != 0 on start, 0 on stop. */
static void identify_cb(zb_bufid_t bufid)
{
	effect_start(bufid ? EFFECT_ID_IDENTIFY : ZB_ZCL_IDENTIFY_EFFECT_ID_STOP);
}

/* ==========================================================================
 * Battery Measurement - LiPo via external divider
 * ========================================================================== */

/*
 * LiPo voltage to percentage lookup table (typical discharge curve, mV).
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

	for (size_t i = 0; i < BATTERY_CURVE_SIZE - 1; i++) {
		if (mv >= lipo_discharge_curve[i + 1].mv) {
			uint16_t v_high = lipo_discharge_curve[i].mv;
			uint16_t v_low = lipo_discharge_curve[i + 1].mv;
			uint8_t p_high = lipo_discharge_curve[i].percent;
			uint8_t p_low = lipo_discharge_curve[i + 1].percent;

			return p_low + ((mv - v_low) * (p_high - p_low)) / (v_high - v_low);
		}
	}

	return 0;
}

/* Battery voltage filtering */
#define BATTERY_OVERSAMPLE_SHIFT  3     /* 2^3 = 8x hardware oversampling */
#define BATTERY_SOFTWARE_SAMPLES  4     /* Additional software averaging */
#define BATTERY_FILTER_ALPHA      64    /* EMA alpha (0-256), 64 = ~25% new value */

static const struct device *adc_dev;
static struct k_work_delayable battery_work;
static uint16_t battery_filtered_mv;    /* Exponential moving average result */
/* Restart the average with the next sample (after charging) */
static bool battery_filter_reset;
/* Send a report to the coordinator after the next measurement */
static atomic_t battery_report_pending;

/**
 * Measure battery voltage using external voltage divider with filtering.
 *
 * Hardware: VBATT -> 1MΩ -> P0.31 (AIN7) -> 1MΩ -> P0.29
 * - Drive P0.29 LOW to enable divider (creates path to GND)
 * - Measure VBATT/2 on P0.31 (AIN7)
 * - Set P0.29 to high-impedance when done so the divider draws no current
 *
 * Returns filtered battery voltage in millivolts, 0 on failure.
 */
static uint16_t battery_measure_mv(void)
{
	int16_t sample;
	int32_t sample_sum = 0;
	uint16_t voltage_mv;
	int ret;

	if (!adc_dev) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&batt_divider_gnd, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to enable battery divider: %d", ret);
		return 0;
	}

	/* Wait for voltage to settle through high-impedance divider */
	k_usleep(500);

	/*
	 * Internal 0.6 V reference, gain 1/6 -> 3.6 V full scale, 12-bit.
	 * Pin sees VBATT/2: VBATT_mV = sample * 3600 * 2 / 4096.
	 */
	struct adc_channel_cfg channel_cfg = {
		.gain = ADC_GAIN_1_6,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = 0,
		.input_positive = NRF_SAADC_INPUT_AIN7,  /* P0.31 */
	};

	ret = adc_channel_setup(adc_dev, &channel_cfg);
	if (ret < 0) {
		LOG_ERR("ADC channel setup failed: %d", ret);
		goto disable_divider;
	}

	struct adc_sequence sequence = {
		.channels = BIT(0),
		.buffer = &sample,
		.buffer_size = sizeof(sample),
		.resolution = 12,
		.oversampling = BATTERY_OVERSAMPLE_SHIFT,
	};

	for (int i = 0; i < BATTERY_SOFTWARE_SAMPLES; i++) {
		/* SAADC offset calibration before the first sample */
		sequence.calibrate = (i == 0);
		ret = adc_read(adc_dev, &sequence);
		if (ret < 0) {
			LOG_ERR("ADC read failed: %d", ret);
			goto disable_divider;
		}
		sample_sum += MAX(sample, 0);
	}

	voltage_mv = (uint32_t)(sample_sum / BATTERY_SOFTWARE_SAMPLES) * 7200U / 4096U;

	if (battery_filtered_mv == 0 || battery_filter_reset) {
		battery_filtered_mv = voltage_mv;
		battery_filter_reset = false;
	} else {
		battery_filtered_mv = ((uint32_t)BATTERY_FILTER_ALPHA * voltage_mv +
			(256 - BATTERY_FILTER_ALPHA) * battery_filtered_mv) / 256;
	}

	LOG_DBG("Battery: raw=%umV, filtered=%umV", voltage_mv, battery_filtered_mv);

disable_divider:
	gpio_pin_configure_dt(&batt_divider_gnd, GPIO_DISCONNECTED);

	return (ret < 0) ? 0 : battery_filtered_mv;
}

static void battery_attrs_store(uint16_t voltage_mv)
{
	dev_ctx.power_config_attr.battery_voltage = voltage_mv / 100;
	dev_ctx.power_config_attr.battery_percentage = battery_mv_to_percent(voltage_mv) * 2;
	dev_ctx.power_config_attr.battery_voltage_raw_mv = voltage_mv;
}

/* Publish a new measurement through ZCL so configured reporting picks it up. */
static void battery_publish(zb_uint8_t param)
{
	ARG_UNUSED(param);

	uint16_t mv = battery_filtered_mv;
	zb_uint8_t voltage = mv / 100;
	zb_uint8_t percentage = battery_mv_to_percent(mv) * 2;

	ZB_ZCL_SET_ATTRIBUTE(LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
			     ZB_ZCL_CLUSTER_SERVER_ROLE,
			     ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
			     &voltage, ZB_FALSE);
	ZB_ZCL_SET_ATTRIBUTE(LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
			     ZB_ZCL_CLUSTER_SERVER_ROLE,
			     ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
			     &percentage, ZB_FALSE);
	ZB_ZCL_SET_ATTRIBUTE(LIGHT_ENDPOINT, ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
			     ZB_ZCL_CLUSTER_SERVER_ROLE,
			     ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_RAW_MV_ID,
			     (zb_uint8_t *)&mv, ZB_FALSE);

	LOG_INF("Battery: %u mV (%u%%)", mv, percentage / 2);
}

/* Zigbee thread: switch the light off while the battery is low. */
static void battery_low_force_off(zb_uint8_t param)
{
	ARG_UNUSED(param);

	if (atomic_get(&battery_low) && dev_ctx.on_off_attr.on_off) {
		on_off_set_value(ZB_FALSE);
	}
}

/* Zigbee thread: send current battery attributes to the coordinator. */
static void battery_report_send(zb_bufid_t bufid)
{
	zb_uint8_t *ptr = ZB_ZCL_START_PACKET(bufid);
	zb_uint16_t coordinator = 0x0000;

	ZB_ZCL_CONSTRUCT_SET_FRAME_CONTROL(*ptr, ZB_ZCL_FRAME_TYPE_COMMON,
					   ZB_ZCL_NOT_MANUFACTURER_SPECIFIC,
					   ZB_ZCL_FRAME_DIRECTION_TO_CLI,
					   ZB_ZCL_DISABLE_DEFAULT_RESPONSE);
	ptr++;
	ZB_ZCL_CONSTRUCT_COMMAND_HEADER(ptr, ZB_ZCL_GET_SEQ_NUM(), ZB_ZCL_CMD_REPORT_ATTRIB);

	ZB_ZCL_PACKET_PUT_DATA16_VAL(ptr, ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID);
	ZB_ZCL_PACKET_PUT_DATA8(ptr, ZB_ZCL_ATTR_TYPE_U8);
	ZB_ZCL_PACKET_PUT_DATA8(ptr, dev_ctx.power_config_attr.battery_voltage);

	ZB_ZCL_PACKET_PUT_DATA16_VAL(ptr, ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID);
	ZB_ZCL_PACKET_PUT_DATA8(ptr, ZB_ZCL_ATTR_TYPE_U8);
	ZB_ZCL_PACKET_PUT_DATA8(ptr, dev_ctx.power_config_attr.battery_percentage);

	ZB_ZCL_PACKET_PUT_DATA16_VAL(ptr, ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_RAW_MV_ID);
	ZB_ZCL_PACKET_PUT_DATA8(ptr, ZB_ZCL_ATTR_TYPE_U16);
	ZB_ZCL_PACKET_PUT_DATA16_VAL(ptr, dev_ctx.power_config_attr.battery_voltage_raw_mv);

	ZB_ZCL_FINISH_N_SEND_PACKET(bufid, ptr, coordinator, ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
				    1, LIGHT_ENDPOINT, ZB_AF_HA_PROFILE_ID,
				    ZB_ZCL_CLUSTER_ID_POWER_CONFIG, NULL);
}

static void battery_report(zb_uint8_t param)
{
	ARG_UNUSED(param);

	if (ZB_JOINED()) {
		zb_buf_get_out_delayed(battery_report_send);
	}
}

/* Workqueue: apply the low-battery hysteresis to a resting measurement. */
static void battery_low_update(uint16_t mv)
{
	if (!atomic_get(&battery_low) && mv < BATTERY_CUTOFF_MV) {
		LOG_WRN("Battery %u mV below cutoff: output off", mv);
		atomic_set(&battery_low, 1);
		ZB_SCHEDULE_APP_CALLBACK(battery_low_force_off, 0);
	} else if (atomic_get(&battery_low) && mv >= BATTERY_RESUME_MV) {
		LOG_INF("Battery %u mV recovered", mv);
		atomic_clear(&battery_low);
	}
}

static void battery_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (nrf_power_usbregstatus_vbusdet_get(NRF_POWER)) {
		/* Charging: the reading would be the charge voltage. */
		battery_filter_reset = true;
		if (atomic_clear(&battery_low)) {
			output_apply(fade_now());
		}
	} else {
		/* Measure at rest: blank a lit strip for the ~2 ms of sampling. */
		bool lit = output_enabled;
		uint16_t mv;

		if (lit) {
			output_apply(0);
		}
		mv = battery_measure_mv();

		if (mv > 0) {
			battery_low_update(mv);
			ZB_SCHEDULE_APP_CALLBACK(battery_publish, 0);
		} else {
			LOG_WRN("Battery measurement failed");
		}

		if (lit) {
			output_apply(fade_now());
		}
	}

	if (atomic_clear(&battery_report_pending)) {
		ZB_SCHEDULE_APP_CALLBACK(battery_report, 0);
	}

	k_work_schedule(&battery_work, K_SECONDS(BATTERY_REPORT_INTERVAL_SEC));
}

/* Zigbee thread: measure and report shortly after (re)joining. */
static void battery_report_after_join(void)
{
	atomic_set(&battery_report_pending, 1);
	k_work_reschedule(&battery_work, K_SECONDS(BATTERY_JOIN_REPORT_DELAY_SEC));
}

/* Must run before the Zigbee stack starts: writes attributes directly. */
static void battery_init(void)
{
	if (!gpio_is_ready_dt(&batt_divider_gnd) ||
	    gpio_pin_configure_dt(&batt_divider_gnd, GPIO_DISCONNECTED) < 0) {
		LOG_WRN("Battery divider GPIO unavailable");
		return;
	}

	adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
	if (!device_is_ready(adc_dev)) {
		LOG_WRN("ADC device not ready");
		adc_dev = NULL;
		return;
	}

	dev_ctx.power_config_attr.battery_size = ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_OTHER; /* LiPo */
	dev_ctx.power_config_attr.battery_quantity = 1;
	dev_ctx.power_config_attr.battery_rated_voltage = 37; /* 3.7V nominal in 100mV units */
	dev_ctx.power_config_attr.battery_alarm_mask = 0;
	dev_ctx.power_config_attr.battery_voltage_min_threshold = 30; /* 3.0V in 100mV units */

	/* Valid values before the coordinator interviews the device */
	uint16_t voltage_mv = battery_measure_mv();

	if (voltage_mv > 0) {
		battery_attrs_store(voltage_mv);
		LOG_INF("Initial battery: %u mV", voltage_mv);

		if (nrf_power_usbregstatus_vbusdet_get(NRF_POWER)) {
			/* Charge voltage: replace with the first resting sample */
			battery_filter_reset = true;
		} else if (voltage_mv < BATTERY_CUTOFF_MV) {
			LOG_WRN("Battery %u mV below cutoff: output off", voltage_mv);
			atomic_set(&battery_low, 1);
		}
	}

	k_work_schedule(&battery_work, K_SECONDS(BATTERY_REPORT_INTERVAL_SEC));
}

/* ==========================================================================
 * Status LED - blinks while not joined (bounded), off otherwise
 * ========================================================================== */

static struct k_work_delayable status_led_work;
static atomic_t zb_joined;
static atomic_t blink_deadline;  /* k_uptime_get_32() value */

static void status_led_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int32_t remaining = (int32_t)((uint32_t)atomic_get(&blink_deadline) - k_uptime_get_32());

	if (atomic_get(&zb_joined) || remaining <= 0) {
		gpio_pin_set_dt(&status_led, 0);
		return;
	}

	gpio_pin_toggle_dt(&status_led);
	k_work_schedule(&status_led_work, K_MSEC(JOIN_BLINK_PERIOD_MS));
}

/* (Re)start join-indication blinking. Callable from any thread. */
static void status_led_blink(void)
{
	atomic_set(&blink_deadline, (atomic_val_t)(k_uptime_get_32() + JOIN_BLINK_TIMEOUT_MS));
	k_work_reschedule(&status_led_work, K_NO_WAIT);
}

static void joined_state_update(void)
{
	bool joined = ZB_JOINED();

	if (atomic_set(&zb_joined, joined) != joined) {
		LOG_INF("Network %s", joined ? "joined" : "left");
		if (joined) {
			k_work_reschedule(&status_led_work, K_NO_WAIT);
		} else {
			status_led_blink();
		}
	}
}

/* ==========================================================================
 * Button Handling
 * ========================================================================== */

static struct gpio_callback button_cb_data;
static struct k_work_delayable button_work;
static struct k_work_delayable long_press_work;
static bool button_pressed;
static bool long_press_fired;

/* Zigbee thread: short press toggles the light and wakes rejoin if needed. */
static void button_short_press(zb_uint8_t param)
{
	ARG_UNUSED(param);

	zb_bool_t on = !dev_ctx.on_off_attr.on_off;

	if (!ZB_JOINED()) {
		user_input_indicate();
		status_led_blink();
	}

	LOG_INF("Button: toggle %s", on ? "ON" : "OFF");
	on_off_set_value(on);
}

/* Zigbee thread: long press leaves the network and wipes Zigbee data. */
static void button_long_press(zb_uint8_t param)
{
	ARG_UNUSED(param);

	LOG_INF("Button: factory reset");
	zb_bdb_reset_via_local_action(0);
	status_led_blink();
}

static void button_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	bool pressed = (gpio_pin_get_dt(&button) == 1);

	if (pressed == button_pressed) {
		return;
	}
	button_pressed = pressed;

	if (pressed) {
		long_press_fired = false;
		k_work_schedule(&long_press_work, K_MSEC(BUTTON_LONG_PRESS_MS));
	} else {
		k_work_cancel_delayable(&long_press_work);
		if (!long_press_fired) {
			ZB_SCHEDULE_APP_CALLBACK(button_short_press, 0);
		}
	}
}

static void long_press_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (button_pressed) {
		long_press_fired = true;
		ZB_SCHEDULE_APP_CALLBACK(button_long_press, 0);
	}
}

static void button_gpio_handler(const struct device *dev,
				struct gpio_callback *cb,
				uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Debounce: sample the pin once it has been quiet for a while */
	k_work_reschedule(&button_work, K_MSEC(BUTTON_DEBOUNCE_MS));
}

static int button_init(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Button device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&button_cb_data, button_gpio_handler, BIT(button.pin));
	ret = gpio_add_callback(button.port, &button_cb_data);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
}

/* ==========================================================================
 * Watchdog
 * ========================================================================== */

static const struct device *const wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));
static int wdt_channel = -1;
static struct k_work_delayable wdt_work;
static atomic_t wdt_zb_alive = ATOMIC_INIT(1);

/* Zigbee thread: answers the ping from the workqueue. */
static void wdt_zb_ping(zb_uint8_t param)
{
	ARG_UNUSED(param);

	atomic_set(&wdt_zb_alive, 1);
}

/*
 * Workqueue: feed only if the Zigbee thread answered the previous ping, so a
 * stall of either thread stops the feeding.
 */
static void wdt_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_clear(&wdt_zb_alive)) {
		wdt_feed(wdt_dev, wdt_channel);
	}

	ZB_SCHEDULE_APP_CALLBACK(wdt_zb_ping, 0);
	k_work_schedule(&wdt_work, K_MSEC(WDT_FEED_INTERVAL_MS));
}

static void watchdog_init(void)
{
	struct wdt_timeout_cfg cfg = {
		.window.max = WDT_TIMEOUT_MS,
		.flags = WDT_FLAG_RESET_SOC,
	};

	if (!device_is_ready(wdt_dev)) {
		LOG_ERR("Watchdog not ready");
		return;
	}

	wdt_channel = wdt_install_timeout(wdt_dev, &cfg);
	if (wdt_channel < 0 || wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG) < 0) {
		LOG_ERR("Watchdog setup failed: %d", wdt_channel);
		wdt_channel = -1;
	}
}

/*
 * Start the ping/feed cycle once the stack finished BDB initialization, so a
 * stack that never starts is reset. The 60 s timeout covers startup.
 */
static void watchdog_start(void)
{
	if (wdt_channel >= 0 && !k_work_delayable_is_pending(&wdt_work)) {
		k_work_schedule(&wdt_work, K_NO_WAIT);
	}
}

/* ==========================================================================
 * USB plug-in -> reboot into the MCUboot USB update window
 * ========================================================================== */

static struct k_work_delayable usb_reboot_work;
/* Set once USB power has been seen absent, so only a real plug-in counts. */
static atomic_t usb_was_absent;

static void usb_reboot_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!nrf_power_usbregstatus_vbusdet_get(NRF_POWER)) {
		return;
	}

	LOG_INF("USB connected: rebooting into bootloader update window");
	k_work_cancel_delayable(&save_work);
	save_work_handler(NULL);
	sys_reboot(SYS_REBOOT_COLD);
}

static void usb_power_handler(nrfx_power_usb_evt_t event)
{
	if (event == NRFX_POWER_USB_EVT_DETECTED) {
		if (atomic_get(&usb_was_absent)) {
			k_work_reschedule(&usb_reboot_work, K_MSEC(USB_REBOOT_DELAY_MS));
		}
		/* Releases the low-battery cutoff */
		k_work_reschedule(&battery_work, K_NO_WAIT);
	} else if (event == NRFX_POWER_USB_EVT_REMOVED) {
		atomic_set(&usb_was_absent, 1);
		k_work_cancel_delayable(&usb_reboot_work);
		atomic_set(&battery_report_pending, 1);
		k_work_reschedule(&battery_work, K_SECONDS(BATTERY_AFTER_USB_DELAY_SEC));
	}
}

/* Only plugging USB in after it was absent triggers a reboot. */
static void usb_detect_init(void)
{
	/* Keep the regulator configuration set up at boot */
	const nrfx_power_config_t power_cfg = {
		.dcdcen = nrf_power_dcdcen_get(NRF_POWER),
		.dcdcenhv = nrf_power_dcdcen_vddh_get(NRF_POWER),
	};
	const nrfx_power_usbevt_config_t usbevt_cfg = {
		.handler = usb_power_handler,
	};

	(void)nrfx_power_init(&power_cfg);
	atomic_set(&usb_was_absent, !nrf_power_usbregstatus_vbusdet_get(NRF_POWER));
	nrfx_power_usbevt_init(&usbevt_cfg);
	nrf_power_event_clear(NRF_POWER, NRF_POWER_EVENT_USBDETECTED);
	nrf_power_event_clear(NRF_POWER, NRF_POWER_EVENT_USBREMOVED);
	nrf_power_event_clear(NRF_POWER, NRF_POWER_EVENT_USBPWRRDY);
	nrfx_power_usbevt_enable();
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

	/* Initialize sw_ver to empty string (first byte = length = 0) */
	dev_ctx.basic_attr.sw_ver[0] = 0;

	dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	/* On/Off attributes */
	dev_ctx.on_off_attr.on_off = ZB_ZCL_ON_OFF_IS_OFF;
	dev_ctx.on_off_attr.global_scene_ctrl = ZB_TRUE;
	dev_ctx.on_off_attr.on_time = 0;
	dev_ctx.on_off_attr.off_wait_time = 0;
	dev_ctx.on_off_attr.start_up_on_off = ZB_ZCL_ON_OFF_STARTUP_PREVIOUS;

	/* Level Control attributes */
	dev_ctx.level_control_attr.current_level = LEVEL_MAX;
	dev_ctx.level_control_attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;
	dev_ctx.level_control_attr.options = 0;
	dev_ctx.level_control_attr.on_off_transition_time = ON_OFF_TRANSITION_DEFAULT;
	dev_ctx.level_control_attr.start_up_current_level = ZB_ZCL_LEVEL_STARTUP_PREVIOUS;
}

/* ==========================================================================
 * Startup Behavior
 * ========================================================================== */

/* Runs after settings_load(), before the Zigbee stack starts. */
static void apply_startup_behavior(void)
{
	zb_uint8_t level;
	zb_bool_t on;

	switch (dev_ctx.level_control_attr.start_up_current_level) {
	case ZB_ZCL_LEVEL_STARTUP_MINIMUM:
		level = LEVEL_MIN;
		break;
	case ZB_ZCL_LEVEL_STARTUP_PREVIOUS:
		level = dev_ctx.level_control_attr.current_level;
		break;
	default:
		level = dev_ctx.level_control_attr.start_up_current_level;
		break;
	}

	switch (dev_ctx.on_off_attr.start_up_on_off) {
	case ZB_ZCL_ON_OFF_STARTUP_OFF:
		on = ZB_FALSE;
		break;
	case ZB_ZCL_ON_OFF_STARTUP_ON:
		on = ZB_TRUE;
		break;
	case ZB_ZCL_ON_OFF_STARTUP_TOGGLE:
		on = !dev_ctx.on_off_attr.on_off;
		break;
	case ZB_ZCL_ON_OFF_STARTUP_PREVIOUS:
	default:
		on = dev_ctx.on_off_attr.on_off ? ZB_TRUE : ZB_FALSE;
		break;
	}

	dev_ctx.level_control_attr.current_level = CLAMP(level, LEVEL_MIN, LEVEL_MAX);
	dev_ctx.on_off_attr.on_off = on;

	fade_to(state_target_lightness(), 0);

	LOG_INF("Startup state: %s, level: %u", on ? "ON" : "OFF",
		dev_ctx.level_control_attr.current_level);
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
		if (!ota_active) {
			/* Download over a slow-polling link would take hours */
			ota_active = true;
			poll_update();
		}
		break;

	case ZIGBEE_FOTA_EVT_FINISHED:
		LOG_INF("OTA download complete, rebooting...");
		sys_reboot(SYS_REBOOT_COLD);
		break;

	case ZIGBEE_FOTA_EVT_ERROR:
		LOG_ERR("OTA transfer failed");
		ota_active = false;
		poll_update();
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

	case ZB_ZCL_SET_ATTR_VALUE_CB_ID: {
		zb_uint16_t cluster_id = param->cb_param.set_attr_value_param.cluster_id;
		zb_uint16_t attr_id = param->cb_param.set_attr_value_param.attr_id;

		if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF &&
		    attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			on_off_set_value(
				param->cb_param.set_attr_value_param.values.data8 ?
				ZB_TRUE : ZB_FALSE);
		} else if (cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL &&
			   attr_id == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID) {
			level_control_set_value(
				param->cb_param.set_attr_value_param.values.data8);
		} else if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF ||
			   cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
			/* StartUpOnOff, StartUpCurrentLevel, OnOffTransitionTime */
			state_save_schedule();
		}
		break;
	}

	case ZB_ZCL_IDENTIFY_EFFECT_CB_ID:
		effect_start(param->cb_param.identify_effect_value_param.effect_id);
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

void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t *sig_hdr = NULL;
	zb_zdo_app_signal_type_t sig_type = zb_get_app_signal(bufid, &sig_hdr);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);
#ifdef CONFIG_ZIGBEE_FOTA
	zigbee_fota_signal_handler(bufid);
#endif

	switch (sig_type) {
	case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
		watchdog_start();
		battery_low_force_off(0);
		if (sig_type == ZB_BDB_SIGNAL_DEVICE_REBOOT && status == RET_OK) {
			battery_report_after_join();
		}
		break;

	case ZB_BDB_SIGNAL_STEERING:
		if (status == RET_OK) {
			battery_report_after_join();
			/* Newly joined: poll fast while the coordinator interviews us */
			interview_window = true;
			ZB_SCHEDULE_APP_ALARM_CANCEL(interview_window_end, ZB_ALARM_ANY_PARAM);
			ZB_SCHEDULE_APP_ALARM(interview_window_end, 0,
				ZB_MILLISECONDS_TO_BEACON_INTERVAL(INTERVIEW_FAST_POLL_MS));
		}
		break;

	case ZB_ZDO_SIGNAL_LEAVE:
		interview_window = false;
		break;

	default:
		break;
	}

	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	if (sig_type != ZB_COMMON_SIGNAL_CAN_SLEEP) {
		joined_state_update();
		poll_update();
	}

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

	ret = output_init();
	if (ret < 0) {
		return ret;
	}

	if (gpio_is_ready_dt(&status_led)) {
		gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
	}

	ret = button_init();
	if (ret < 0) {
		LOG_ERR("Button init failed: %d", ret);
		return ret;
	}

	battery_init();
	usb_detect_init();

	return 0;
}

/* ==========================================================================
 * Main
 * ========================================================================== */

int main(void)
{
	int err;

	LOG_INF("LED Copper String Controller, board %s", CONFIG_BOARD);

	k_work_init_delayable(&fade_work, fade_work_handler);
	k_work_init_delayable(&effect_work, effect_work_handler);
	k_work_init_delayable(&save_work, save_work_handler);
	k_work_init_delayable(&battery_work, battery_work_handler);
	k_work_init_delayable(&status_led_work, status_led_work_handler);
	k_work_init_delayable(&button_work, button_work_handler);
	k_work_init_delayable(&long_press_work, long_press_work_handler);
	k_work_init_delayable(&usb_reboot_work, usb_reboot_work_handler);
	k_work_init_delayable(&wdt_work, wdt_work_handler);

	watchdog_init();

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
	err = zigbee_fota_init(fota_evt_handler);
	if (err) {
		LOG_ERR("FOTA init failed: %d", err);
	}

	/* Mark the running image as good so MCUboot does not revert it */
	if (!boot_is_img_confirmed()) {
		err = boot_write_img_confirmed();
		if (err) {
			LOG_ERR("Failed to confirm image: %d", err);
		}
	}
#endif

	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);
	ZB_AF_REGISTER_DEVICE_CTX(&light_ctx);
	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(LIGHT_ENDPOINT, identify_cb);
	ZB_AF_SET_ENDPOINT_HANDLER(LIGHT_ENDPOINT, light_ep_handler);

	clusters_attr_init();

	/* Restores on/off, level and the writable configuration attributes */
	err = settings_load();
	if (err) {
		LOG_ERR("Settings load failed: %d", err);
	}

	apply_startup_behavior();

	/* Join indication until the stack reports a network */
	status_led_blink();

	/*
	 * Sleepy End Device: radio off between polls. The parent keeps us in
	 * its child table for 64 min without contact.
	 */
	zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
	zigbee_configure_sleepy_behavior(true);

	if (IS_ENABLED(CONFIG_RAM_POWER_DOWN_LIBRARY)) {
		power_down_unused_ram();
	}

	zigbee_enable();

	LOG_INF("Hold button 3 s to reset and pair");

	return 0;
}
