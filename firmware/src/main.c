/**
 * @file main.c
 * @brief LED Copper String Zigbee Controller
 *
 * Single PWM output on P0.10 (1 kHz, active high), brightness set by its
 * duty cycle. Generated in hardware; at 0 the PWM peripheral is stopped.
 *
 * Threading:
 * - All PWM output changes run on the system workqueue.
 * - All ZBOSS API calls run in the Zigbee thread; other contexts hand work
 *   over with ZB_SCHEDULE_APP_CALLBACK (thread-safe in NCS).
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>
#include <ram_pwrdn.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zb_nrf_platform.h>
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
#define BULB_INIT_BASIC_MODEL_ID        "LEDCopperSimple"
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

/* Delay before persisting state, so bursts of changes cost one flash write. */
#define STATE_SAVE_DELAY_MS             2000

/*
 * Watchdog: reset if the Zigbee thread or the system workqueue stops running.
 * The workqueue feeds only after the Zigbee thread answered its last ping.
 */
#define WDT_TIMEOUT_MS                  60000
#define WDT_FEED_INTERVAL_MS            15000

/* ==========================================================================
 * Device Tree
 * ========================================================================== */

/* LED PWM output (P0.10) */
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_NODELABEL(led_pwm));

/* Onboard status LED (P0.15), second channel of the same PWM instance */
static const struct pwm_dt_spec pwm_status = PWM_DT_SPEC_GET(DT_NODELABEL(status_pwm));

BUILD_ASSERT(DT_SAME_NODE(DT_PWMS_CTLR(DT_NODELABEL(led_pwm)),
			  DT_PWMS_CTLR(DT_NODELABEL(status_pwm))) &&
	     DT_PWMS_PERIOD(DT_NODELABEL(led_pwm)) == DT_PWMS_PERIOD(DT_NODELABEL(status_pwm)),
	     "LED and status PWM must share one PWM instance and period");

/* Button */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

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

typedef struct {
	zb_zcl_basic_attrs_ext_t     basic_attr;
	zb_zcl_identify_attrs_t      identify_attr;
	zb_zcl_scenes_attrs_t        scenes_attr;
	zb_zcl_groups_attrs_t        groups_attr;
	on_off_attrs_ext_t           on_off_attr;
	level_control_attrs_ext_t    level_control_attr;
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

/* Cluster list - 6 clusters */
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
};

/* Simple descriptor for dimmable light (6 in clusters) */
ZB_DECLARE_SIMPLE_DESC(6, 0);

ZB_AF_SIMPLE_DESC_TYPE(6, 0) simple_desc_light_ep = {
	.endpoint = LIGHT_ENDPOINT,
	.app_profile_id = ZB_AF_HA_PROFILE_ID,
	.app_device_id = ZB_DIMMABLE_LIGHT_DEVICE_ID,
	.app_device_version = ZB_DEVICE_VER_DIMMABLE_LIGHT,
	.reserved = 0,
	.app_input_cluster_count = 6,
	.app_output_cluster_count = 0,
	.app_cluster_list = {
		ZB_ZCL_CLUSTER_ID_BASIC,
		ZB_ZCL_CLUSTER_ID_IDENTIFY,
		ZB_ZCL_CLUSTER_ID_SCENES,
		ZB_ZCL_CLUSTER_ID_GROUPS,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
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
 * PWM Output (system workqueue only)
 * ========================================================================== */

static uint32_t pwm_period_cycles;
/* Pulse currently on the LED output, mirrored on the status LED */
static uint32_t output_pulse;
/* Set while the status LED shows the join blink instead of the mirror */
static bool status_blinking;

static void status_pwm_set(uint32_t pulse)
{
	if (pwm_set_cycles(pwm_status.dev, pwm_status.channel, pwm_period_cycles,
			   pulse, pwm_status.flags)) {
		LOG_ERR("Status PWM set failed");
	}
}

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

	if (!pwm_is_ready_dt(&pwm_led)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}

	ret = pwm_get_cycles_per_sec(pwm_led.dev, pwm_led.channel, &cycles_per_sec);
	if (ret < 0) {
		return ret;
	}
	pwm_period_cycles = (uint32_t)(cycles_per_sec * pwm_led.period / NSEC_PER_SEC);

	LOG_INF("PWM period %u cycles", pwm_period_cycles);
	return 0;
}

/* Drive the output at the given perceptual lightness (0 = off). */
static void output_apply(uint16_t lightness)
{
	uint32_t pulse = cie1931_correct(lightness, pwm_period_cycles);

	if (pwm_set_cycles(pwm_led.dev, pwm_led.channel, pwm_period_cycles,
			   pulse, pwm_led.flags)) {
		LOG_ERR("PWM set failed");
	}

	output_pulse = pulse;
	if (!status_blinking) {
		status_pwm_set(pulse);
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
 * Zigbee-side light control (Zigbee thread only)
 * ========================================================================== */

static int64_t last_level_step_time;

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
	state_save_schedule();
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
 * Status LED - blinks while not joined (bounded), else mirrors the output
 * (system workqueue, same as output_apply)
 * ========================================================================== */

static struct k_work_delayable status_led_work;
static atomic_t zb_joined;
static atomic_t blink_deadline;  /* k_uptime_get_32() value */
static bool status_blink_on;

static void status_led_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int32_t remaining = (int32_t)((uint32_t)atomic_get(&blink_deadline) - k_uptime_get_32());

	if (atomic_get(&zb_joined) || remaining <= 0) {
		status_blinking = false;
		status_pwm_set(output_pulse);
		return;
	}

	status_blinking = true;
	status_blink_on = !status_blink_on;
	status_pwm_set(status_blink_on ? pwm_period_cycles : 0);
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
 * Zigbee Attribute Initialization
 * ========================================================================== */

static void clusters_attr_init(void)
{
	dev_ctx.basic_attr.zcl_version = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.app_version = BULB_INIT_BASIC_APP_VERSION;
	dev_ctx.basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
	dev_ctx.basic_attr.hw_version = BULB_INIT_BASIC_HW_VERSION;
	dev_ctx.basic_attr.power_source = ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;
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
#ifdef CONFIG_ZIGBEE_FOTA
	zigbee_fota_signal_handler(bufid);
#endif

	switch (sig_type) {
	case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
		watchdog_start();
		break;

	default:
		break;
	}

	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	if (sig_type != ZB_COMMON_SIGNAL_CAN_SLEEP) {
		joined_state_update();
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

	ret = button_init();
	if (ret < 0) {
		LOG_ERR("Button init failed: %d", ret);
		return ret;
	}

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
	k_work_init_delayable(&status_led_work, status_led_work_handler);
	k_work_init_delayable(&button_work, button_work_handler);
	k_work_init_delayable(&long_press_work, long_press_work_handler);
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

	/* Rx-on End Device: the parent keeps us for 64 min without contact. */
	zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);

	if (IS_ENABLED(CONFIG_RAM_POWER_DOWN_LIBRARY)) {
		power_down_unused_ram();
	}

	zigbee_enable();

	LOG_INF("Hold button 3 s to reset and pair");

	return 0;
}
