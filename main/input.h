/*
 * input.h - the rotary encoder and its push button.
 *
 * The encoder is counted by an interrupt on every change of A or B, using the
 * same 16-entry lookup table as the wiring test. The count only ever goes up
 * or down by one per valid change; impossible jumps (contact chatter) are
 * counted separately and thrown away.
 *
 * The button is not interrupt-driven: it is sampled every KNOB_POLL_MS and
 * only accepted once it has read the same value for the debounce time.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t input_init(void);

int32_t   input_total(void);          /* running count; clockwise = up          */
uint32_t  input_rejected(void);       /* impossible jumps thrown away           */
uint32_t  input_last_edge_us(void);   /* time of the last valid encoder change  */
void      input_set_reverse(bool reverse);
bool      input_get_reverse(void);

/* Call every KNOB_POLL_MS. Returns true exactly once per debounced press. */
bool      input_button_poll(uint8_t stable_samples_needed);
bool      input_button_is_pressed(void);
