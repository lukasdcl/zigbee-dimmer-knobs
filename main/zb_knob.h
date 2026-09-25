/*
 * zb_knob.h - the Zigbee side: joins the network as a router, exposes one
 * Dimmer Switch endpoint per knob that Zigbee2MQTT can bind to bulbs, and
 * turns each knob's encoder counts + button presses into BOUND commands (no
 * destination address in the firmware - the stack sends to whatever the
 * binding table says for that knob's endpoint).
 *
 * Every function that takes `knob` counts from 0 (0 = first row of
 * KNOB_WIRING). The console converts from the 1-based numbers you type.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Live-tunable settings. Each knob has its own copy, changed by the `tune`
 * console command and saved to flash per knob. */
typedef struct {
    uint16_t    tick_ms;          /* minimum gap between dimming commands     */
    uint8_t     units;            /* brightness units per encoder count       */
    uint8_t     fade_ds;          /* fade time per command, tenths of second  */
    uint8_t     settle_fade_ds;   /* fade time of the landing command         */
    uint16_t    settle_ms;        /* stillness before the landing command     */
    uint8_t     floor_level;      /* below this, land on minimum instead      */
    uint16_t    debounce_ms;      /* button must be stable this long          */
    bool        log_tx;           /* print every command sent                 */
    uint8_t     rev_counts;       /* opposite counts needed to reverse; 0=off */
    bool        settle;           /* send the landing command at all          */
} knob_tuning_t;

/* That knob's live settings. The console edits them through this pointer,
 * then calls zb_knob_save_settings(knob). */
knob_tuning_t *zb_knob_tuning(int knob);

esp_err_t zb_knob_start(void);

/* Console helpers for one knob - all run inside the Zigbee task, so they're
 * safe to call from anywhere. They return an error if the stack isn't
 * running yet. */
esp_err_t zb_knob_toggle(int knob);
esp_err_t zb_knob_onoff(int knob, bool on);
esp_err_t zb_knob_step(int knob, bool up, uint8_t size);
void      zb_knob_print_stats(int knob);
void      zb_knob_print_state(int knob);
void      zb_knob_save_settings(int knob);    /* keep current tuning across reboots */
void      zb_knob_forget_settings(int knob);  /* back to the built-in defaults      */

/* Whole-device helpers (one radio, one network, shared by all knobs). */
esp_err_t zb_knob_print_info(void);
esp_err_t zb_knob_steer(void);
esp_err_t zb_knob_factory_reset(void);
