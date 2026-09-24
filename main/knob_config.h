/*
 * knob_config.h - every setting you might want to change, in one place.
 *
 * The dimming values here are only the STARTING values. All of them can be
 * changed live over the serial console with the `tune` command, so you can
 * experiment without rebuilding. When you find settings you like, copy them
 * back in here so they survive a reboot.
 */
#pragma once

/* ---------------- Wiring (ESP32-C6-DevKitC-1, tested 22 Sep) ----------------
 * Encoder middle pin + one switch pin -> GND. Nothing else. */
#define KNOB_GPIO_ENC_A                 18
#define KNOB_GPIO_ENC_B                 19
#define KNOB_GPIO_BUTTON                20

/* 1 = use the chip's built-in pull-ups (bench testing, short wires).
 * 0 = external 2.2k pull-ups fitted (final install on the long Cat6 run). */
#define KNOB_USE_INTERNAL_PULLUPS       1

/* 1 flips the direction. Also changeable live: `tune reverse 1`. */
#define KNOB_ENCODER_REVERSE_DEFAULT    0

/* ---------------- Zigbee network ----------------
 * By default this scans every standard 2.4GHz Zigbee channel (11-26) while
 * joining, same as any normal Zigbee router - it'll find your network
 * whatever channel your coordinator picked, no editing required.
 *
 * If you know your coordinator's channel and want faster, more reliable
 * joining (fewer channels to scan = less chance of picking up a neighbour's
 * network first), lock to it: comment out KNOB_ZB_CHANNEL_MASK below and
 * uncomment the single-channel version instead. */
#define KNOB_ZB_CHANNEL_MASK            0x07FFF800UL   /* channels 11-26 */
/* #define KNOB_ZB_CHANNEL               13
 * #define KNOB_ZB_CHANNEL_MASK          (1UL << KNOB_ZB_CHANNEL) */

#define KNOB_EP                         1      /* commands out: bind this to bulbs  */
#define KNOB_EP_REPORTS                 2      /* reports in: bind bulbs back here  */
#define KNOB_MAX_CHILDREN               10
#define KNOB_COMMISSION_RETRY_MS        5000

/* Identity shown in Zigbee2MQTT. ZCL strings start with a length byte. */
#define KNOB_MANUFACTURER_NAME          "\x08" "DIYKnobs"         /*  8 chars */
#define KNOB_MODEL_IDENTIFIER           "\x0F" "dimmer.knob.poc"  /* 15 chars */

/* ---------------- Dimming (starting values, all live-tunable) ---------------- */
#define KNOB_DEFAULT_TICK_MS            250    /* never send faster than this        */
#define KNOB_DEFAULT_UNITS              3      /* brightness units per encoder count */
#define KNOB_DEFAULT_FADE_DS            3      /* bulb fade per step, tenths of a s  */
#define KNOB_DEFAULT_SETTLE_FADE_DS     10     /* landing fade: 1.0 s, eases in     */
#define KNOB_DEFAULT_SETTLE_MS          150    /* stillness before the landing cmd   */
#define KNOB_DEFAULT_FLOOR              8      /* below this, go straight to minimum */
#define KNOB_DEFAULT_DEBOUNCE_MS        30     /* button must be stable this long    */
#define KNOB_DEFAULT_LOG_TX             0      /* print every command sent           */

#define KNOB_DEFAULT_REV_COUNTS         8      /* opposite counts to reverse (8 = 2 clicks) */

/* Some bulbs (Philips Hue) ignore "go to level 1 and switch on". 1 = follow it
 * with a plain On and a plain "go to minimum" 200 ms apart. */
#define KNOB_WAKE_SEQUENCE              1

/* Reports from bulbs (endpoint 2). If the build ever fails inside the report
 * handler, set this to 0: everything else keeps working without reports. */
#define KNOB_USE_REPORTS                1
/* 2 = message->info / message->in (SDK 2.x style), 1 = flat older style.
 * Only change if the compiler complains about fields in the report handler. */
#define KNOB_REPORT_STRUCT_STYLE        2

/* How often the input poller runs. Leave at 10. */
#define KNOB_POLL_MS                    10

/* ---------------- Production board (Seeed XIAO ESP32C6) ----------------
 * Not used on the DevKitC-1. Uncomment for the XIAO build: GPIO3/GPIO14 then
 * drive its antenna switch and must not be reused. */
/* #define BOARD_XIAO_ESP32C6 */
#ifdef BOARD_XIAO_ESP32C6
#define XIAO_GPIO_RF_SWITCH_EN          3      /* LOW enables the RF switch      */
#define XIAO_GPIO_ANT_SELECT            14     /* HIGH = external U.FL antenna   */
#endif
