/*
 * knob_config.h - every setting you might want to change, in one place.
 *
 * The dimming values here are only the STARTING values. All of them can be
 * changed live over the serial console with the `tune` command, so you can
 * experiment without rebuilding. Changes made with `tune` are saved to flash
 * automatically, separately for each knob.
 */
#pragma once

#include <stdint.h>

/* ---------------- Production board (Seeed XIAO ESP32C6) ----------------
 * Not used on the DevKitC-1. Uncomment for the XIAO build: GPIO3/GPIO14 then
 * drive its antenna switch and must not be reused. This also picks the XIAO
 * wiring table below instead of the DevKit one. */
/* #define BOARD_XIAO_ESP32C6 */
#ifdef BOARD_XIAO_ESP32C6
#define XIAO_GPIO_RF_SWITCH_EN          3      /* LOW enables the RF switch      */
#define XIAO_GPIO_ANT_SELECT            14     /* HIGH = external U.FL antenna   */
#endif

/* =====================================================================
 * THE WIRING TABLE - one row per physical knob.
 *
 * To add a knob: uncomment (or add) a row, wire the encoder to those pins,
 * rebuild. Nothing else in the code needs touching. The firmware checks the
 * table at start-up and refuses to run (with a clear message on the serial
 * console) if two knobs share a pin or an endpoint, or a pin is one the
 * board needs for something else.
 *
 *   enc_a, enc_b : the encoder's two outer pins on the 3-pin side
 *   button       : one pin of the push switch (the other pins go to GND)
 *   cmd_ep       : Zigbee endpoint this knob's commands go OUT from.
 *                  In Z2M, bind THIS endpoint to the bulbs the knob controls.
 *   report_ep    : Zigbee endpoint that RECEIVES reports from those bulbs
 *                  (optional - see README). 0 = this knob has none.
 *
 * Knobs are numbered from 1 in the order of the rows: the first row is
 * "knob 1" on the console, the second "knob 2", and so on.
 *
 * Endpoint numbers must be 1-240 and all different. The pattern below
 * (knob n -> endpoints 2n-1 and 2n) is just a convention.
 *
 * Changing the number of knobs changes the endpoints this device offers, so
 * after reflashing, Zigbee2MQTT needs to re-read it: in Z2M open the device
 * and press "Reconfigure" (or remove it and let it re-join). Existing
 * bindings on endpoints that still exist are kept by the knob itself.
 * ===================================================================== */
typedef struct {
    int     enc_a, enc_b, button;   /* GPIO numbers                        */
    uint8_t cmd_ep, report_ep;      /* Zigbee endpoints (report_ep 0 = none) */
} knob_wiring_t;

#ifndef BOARD_XIAO_ESP32C6
/* ---- ESP32-C6-DevKitC-1 (knob 1 tested 22 Sep) ----
 * Free pins on this board: 0-7, 10, 11, 18-23. Avoid 8, 9, 15 (they decide
 * how the chip boots), 12/13 (USB) and 16/17 (the serial console). */
static const knob_wiring_t KNOB_WIRING[] = {
    { .enc_a = 18, .enc_b = 19, .button = 20, .cmd_ep = 1, .report_ep = 2 },
 /* { .enc_a = 21, .enc_b = 22, .button = 23, .cmd_ep = 3, .report_ep = 4 }, */
 /* { .enc_a =  0, .enc_b =  1, .button =  2, .cmd_ep = 5, .report_ep = 6 }, */
 /* { .enc_a =  6, .enc_b =  7, .button = 10, .cmd_ep = 7, .report_ep = 8 }, */
};
#else
/* ---- Seeed XIAO ESP32C6 ----
 * The XIAO has 11 pins along its edges (D0-D10). Three knobs fit on those
 * with two to spare. A 4th knob needs 12 pins, which only works if:
 *   - the serial console moves to the USB port (see README, "XIAO console"),
 *     which frees D6/D7 (GPIO16/17), AND
 *   - one pin comes from the small solder pads on the UNDERSIDE of the board
 *     (MTCK = GPIO6 used below; MTDO = GPIO7 would also do).
 * GPIO3 and GPIO14 are the antenna switch, GPIO15 the LED and GPIO9 the
 * BOOT button - none of those are on the edge pads anyway. */
static const knob_wiring_t KNOB_WIRING[] = {
    { .enc_a =  0, .enc_b =  1, .button =  2, .cmd_ep = 1, .report_ep = 2 },  /* D0 D1 D2   */
 /* { .enc_a = 21, .enc_b = 22, .button = 23, .cmd_ep = 3, .report_ep = 4 }, */ /* D3 D4 D5  */
 /* { .enc_a = 19, .enc_b = 20, .button = 18, .cmd_ep = 5, .report_ep = 6 }, */ /* D8 D9 D10 */
 /* { .enc_a = 16, .enc_b = 17, .button =  6, .cmd_ep = 7, .report_ep = 8 }, */ /* D6 D7 +underside MTCK; USB console only */
};
#endif

/* How many rows the table has - worked out by the compiler, never edit. */
#define KNOB_COUNT   ((int)(sizeof(KNOB_WIRING) / sizeof(KNOB_WIRING[0])))
#define KNOB_MAX     8    /* sanity limit; the XIAO physically can't do more than 4 */
_Static_assert(KNOB_COUNT >= 1 && KNOB_COUNT <= KNOB_MAX,
               "KNOB_WIRING needs between 1 and KNOB_MAX rows");

/* 1 = use the chip's built-in pull-ups (bench testing, short wires).
 * 0 = external 2.2k pull-ups fitted (final install on the long Cat6 run).
 * Applies to every knob. */
#define KNOB_USE_INTERNAL_PULLUPS       1

/* 1 flips the direction. This is only the starting value: each knob can be
 * flipped on its own, live, with `tune <knob> reverse 1`. */
#define KNOB_ENCODER_REVERSE_DEFAULT    0

/* ---------------- Zigbee network (shared by all knobs) ----------------
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

#define KNOB_MAX_CHILDREN               10
#define KNOB_COMMISSION_RETRY_MS        5000

/* Identity shown in Zigbee2MQTT. ZCL strings start with a length byte. */
#define KNOB_MANUFACTURER_NAME          "\x08" "DIYKnobs"         /*  8 chars */
#define KNOB_MODEL_IDENTIFIER           "\x0F" "dimmer.knob.poc"  /* 15 chars */

/* ---------------- Dimming (starting values for every knob, all live-tunable) ---------------- */
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

/* Reports from bulbs (each knob's report_ep). If the build ever fails inside
 * the report handler, set this to 0: everything else keeps working. */
#define KNOB_USE_REPORTS                1
/* 2 = message->info / message->in (SDK 2.x style), 1 = flat older style.
 * Only change if the compiler complains about fields in the report handler. */
#define KNOB_REPORT_STRUCT_STYLE        2

/* How often the input poller runs (one timer checks every knob). Leave at 10. */
#define KNOB_POLL_MS                    10
