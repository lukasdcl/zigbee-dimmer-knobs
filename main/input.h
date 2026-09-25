/*
 * input.h - the rotary encoders and their push buttons.
 *
 * One instance per row of KNOB_WIRING (knob_config.h). Every function takes
 * a knob number `knob`, counted from 0 in the code (0 = the first row). The
 * console shows these to you counted from 1, which is the only place that
 * conversion happens.
 *
 * Each encoder is counted by an interrupt on every change of its A or B pin,
 * using the same 16-entry lookup table as the wiring test. The count only
 * ever goes up or down by one per valid change; impossible jumps (contact
 * chatter) are counted separately and thrown away.
 *
 * The buttons are not interrupt-driven: each is sampled every KNOB_POLL_MS
 * and only accepted once it has read the same value for the debounce time.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Sets up every knob in the wiring table. Fails (and says why on the
 * console) if the table has a problem, e.g. two knobs on the same pin. */
esp_err_t input_init(void);

int32_t   input_total(int knob);          /* running count; clockwise = up          */
uint32_t  input_rejected(int knob);       /* impossible jumps thrown away           */
uint32_t  input_last_edge_us(int knob);   /* time of the last valid encoder change  */
void      input_set_reverse(int knob, bool reverse);
bool      input_get_reverse(int knob);

/* Call every KNOB_POLL_MS. Returns true exactly once per debounced press. */
bool      input_button_poll(int knob, uint8_t stable_samples_needed);
bool      input_button_is_pressed(int knob);
