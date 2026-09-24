/*
 * zb_knob.h - the Zigbee side: joins the network as a router, exposes one
 * Dimmer Switch endpoint that Zigbee2MQTT can bind to a bulb, and turns
 * encoder counts + button presses into BOUND commands (no destination address
 * in the firmware - the stack sends to whatever the binding table says).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Live-tunable settings. Changed by the `tune` console command. */
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

extern knob_tuning_t g_knob_tune;

esp_err_t zb_knob_start(void);

/* Console helpers - all run inside the Zigbee task, so they're safe to call
 * from anywhere. They return an error if the stack isn't running yet. */
esp_err_t zb_knob_toggle(void);
esp_err_t zb_knob_onoff(bool on);
esp_err_t zb_knob_step(bool up, uint8_t size);
esp_err_t zb_knob_print_info(void);
esp_err_t zb_knob_steer(void);
esp_err_t zb_knob_factory_reset(void);
void      zb_knob_print_stats(void);
void      zb_knob_print_state(void);
void      zb_knob_save_settings(void);    /* keep current tuning across reboots */
void      zb_knob_forget_settings(void);  /* back to the built-in defaults      */
