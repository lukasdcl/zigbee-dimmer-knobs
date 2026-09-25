/*
 * console_cmds.c - serial console. Type `help` for the list. Everything here
 * also works with Zigbee2MQTT and Home Assistant switched off - commands go
 * straight to the bound bulbs.
 *
 * Choosing a knob: most commands take an optional knob number straight
 * after the command name, counted from 1 (the first row of KNOB_WIRING is
 * knob 1).
 *
 *   on 2 / off 2 / toggle 2 / step 2 up 16
 *       -> knob 2. Without a number these act on knob 1, so they behave
 *          exactly as before on a one-knob build.
 *   state 2 / stats 2 / tune 2 / defaults 2
 *       -> knob 2 only. Without a number: every knob.
 *   tune 2 units 4
 *       -> change knob 2 only.
 *   tune units 4
 *       -> change EVERY knob (most of the time you want them to feel the
 *          same; use a knob number when you don't).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "esp_console.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "knob_config.h"
#include "input.h"
#include "zb_knob.h"

static int report(esp_err_t err)
{
    if (err == ESP_ERR_INVALID_STATE) {
        printf("Zigbee stack not running yet - wait a second and retry\n");
    } else if (err != ESP_OK) {
        printf("failed: %s\n", esp_err_to_name(err));
    }
    return err == ESP_OK ? 0 : 1;
}

/* ------------------------------------------------------------------ */
/* Picking out the knob number                                         */
/* ------------------------------------------------------------------ */

#define KNOB_ALL   (-1)     /* no number given                 */
#define KNOB_BAD   (-2)     /* a number, but not a real knob   */

static bool is_number(const char *s)
{
    if (s == NULL || *s == '\0') {
        return false;
    }
    for (; *s; s++) {
        if (!isdigit((unsigned char)*s)) {
            return false;
        }
    }
    return true;
}

/*
 * If argv[*pos] is a number, it's a knob number: use it up (move *pos on)
 * and return the knob counted from 0. Otherwise return KNOB_ALL and leave
 * *pos alone. Prints a message and returns KNOB_BAD for e.g. "knob 5" on a
 * two-knob build.
 */
static int take_knob(int argc, char **argv, int *pos)
{
    if (*pos >= argc || !is_number(argv[*pos])) {
        return KNOB_ALL;
    }
    int n = atoi(argv[*pos]);
    (*pos)++;
    if (n < 1 || n > KNOB_COUNT) {
        if (KNOB_COUNT == 1) {
            printf("there is only knob 1\n");
        } else {
            printf("no knob %d - knobs are numbered 1 to %d\n", n, KNOB_COUNT);
        }
        return KNOB_BAD;
    }
    return n - 1;
}

/* For commands that act on one knob: no number means knob 1. */
static int take_one_knob(int argc, char **argv, int *pos)
{
    int k = take_knob(argc, argv, pos);
    return k == KNOB_ALL ? 0 : k;
}

/* ------------------------------------------------------------------ */
/* Actions on one knob's bulbs                                         */
/* ------------------------------------------------------------------ */

/* toggle / on / off [knob] */
static int onoff_common(int argc, char **argv, int mode)   /* 0 off, 1 on, 2 toggle */
{
    int pos = 1;
    int k = take_one_knob(argc, argv, &pos);
    if (k == KNOB_BAD) {
        return 1;
    }
    return report(mode == 2 ? zb_knob_toggle(k) : zb_knob_onoff(k, mode == 1));
}
static int cmd_toggle(int argc, char **argv) { return onoff_common(argc, argv, 2); }
static int cmd_on(int argc, char **argv)     { return onoff_common(argc, argv, 1); }
static int cmd_off(int argc, char **argv)    { return onoff_common(argc, argv, 0); }

/* step [knob] <up|down> [size] */
static int cmd_step(int argc, char **argv)
{
    int pos = 1;
    int k = take_one_knob(argc, argv, &pos);
    if (k == KNOB_BAD) {
        return 1;
    }
    if (pos >= argc) {
        printf("usage: step [knob] <up|down> [size 1-254, default 16]\n");
        return 1;
    }
    bool up   = (argv[pos][0] == 'u');
    int  size = (pos + 1 < argc) ? atoi(argv[pos + 1]) : 16;
    if (size < 1)   size = 1;
    if (size > 254) size = 254;
    return report(zb_knob_step(k, up, (uint8_t)size));
}

/* ------------------------------------------------------------------ */
/* Read-outs: one knob, or all of them                                 */
/* ------------------------------------------------------------------ */

/* Runs `fn` for the knob given, or for every knob if none was given. */
static int for_knobs(int argc, char **argv, void (*fn)(int knob))
{
    int pos = 1;
    int k = take_knob(argc, argv, &pos);
    if (k == KNOB_BAD) {
        return 1;
    }
    for (int i = 0; i < KNOB_COUNT; i++) {
        if (k == KNOB_ALL || k == i) {
            fn(i);
        }
    }
    return 0;
}

static int cmd_stats(int argc, char **argv) { return for_knobs(argc, argv, zb_knob_print_stats); }
static int cmd_state(int argc, char **argv) { return for_knobs(argc, argv, zb_knob_print_state); }

static int cmd_info(int argc, char **argv)   { return report(zb_knob_print_info()); }
static int cmd_steer(int argc, char **argv)  { return report(zb_knob_steer()); }
static int cmd_reset(int argc, char **argv)  { return report(zb_knob_factory_reset()); }

/* ------------------------------------------------------------------ */
/* Tuning                                                              */
/*                                                                     */
/* One row per setting: its name, allowed range and description. The   */
/* show/change code below works from this table, so adding a setting   */
/* means adding a row here plus a line in get/set.                     */
/* ------------------------------------------------------------------ */

typedef enum {
    SET_TICK, SET_UNITS, SET_FADE, SET_SETTLE, SET_SFADE, SET_LAND, SET_FLOOR,
    SET_DEBOUNCE, SET_REVERSE, SET_REV, SET_LOG,
} setting_id_t;

typedef struct {
    setting_id_t id;
    const char  *name;
    int          lo, hi;
    const char  *help;
} setting_t;

static const setting_t SETTINGS[] = {
    { SET_TICK,     "tick",     20, 1000, "ms, minimum gap between dimming commands" },
    { SET_UNITS,    "units",     1,   64, "brightness units per encoder count (4 counts per click)" },
    { SET_FADE,     "fade",      0,   50, "tenths of a second each command fades over" },
    { SET_SETTLE,   "settle",   50, 2000, "ms of stillness before the landing command" },
    { SET_SFADE,    "sfade",     0,  100, "tenths of a second the landing command fades over" },
    { SET_LAND,     "land",      0,    1, "1 = send the landing command that re-syncs the bulbs" },
    { SET_FLOOR,    "floor",     1,   60, "below this level, go straight to minimum" },
    { SET_DEBOUNCE, "debounce", 10,  200, "ms the button must be stable" },
    { SET_REVERSE,  "reverse",   0,    1, "1 = swap turning direction" },
    { SET_REV,      "rev",       0,   40, "opposite counts needed to change direction (4 per click); 0 = off" },
    { SET_LOG,      "log",       0,    1, "1 = print every command sent (turn off when judging feel)" },
};
#define SETTING_COUNT ((int)(sizeof(SETTINGS) / sizeof(SETTINGS[0])))

static int get_setting(int knob, setting_id_t id)
{
    const knob_tuning_t *t = zb_knob_tuning(knob);
    switch (id) {
    case SET_TICK:     return t->tick_ms;
    case SET_UNITS:    return t->units;
    case SET_FADE:     return t->fade_ds;
    case SET_SETTLE:   return t->settle_ms;
    case SET_SFADE:    return t->settle_fade_ds;
    case SET_LAND:     return t->settle ? 1 : 0;
    case SET_FLOOR:    return t->floor_level;
    case SET_DEBOUNCE: return t->debounce_ms;
    case SET_REVERSE:  return input_get_reverse(knob) ? 1 : 0;
    case SET_REV:      return t->rev_counts;
    case SET_LOG:      return t->log_tx ? 1 : 0;
    }
    return 0;
}

static void set_setting(int knob, setting_id_t id, int v)
{
    knob_tuning_t *t = zb_knob_tuning(knob);
    switch (id) {
    case SET_TICK:     t->tick_ms        = (uint16_t)v; break;
    case SET_UNITS:    t->units          = (uint8_t)v;  break;
    case SET_FADE:     t->fade_ds        = (uint8_t)v;  break;
    case SET_SETTLE:   t->settle_ms      = (uint16_t)v; break;
    case SET_SFADE:    t->settle_fade_ds = (uint8_t)v;  break;
    case SET_LAND:     t->settle         = (v != 0);    break;
    case SET_FLOOR:    t->floor_level    = (uint8_t)v;  break;
    case SET_DEBOUNCE: t->debounce_ms    = (uint16_t)v; break;
    case SET_REVERSE:  input_set_reverse(knob, v != 0); break;
    case SET_REV:      t->rev_counts     = (uint8_t)v;  break;
    case SET_LOG:      t->log_tx         = (v != 0);    break;
    }
}

/* Prints a table: one row per setting, one column per knob shown.
 * knob = KNOB_ALL shows every knob side by side. */
static void print_tuning(int knob)
{
    printf("%-9s", "");
    for (int i = 0; i < KNOB_COUNT; i++) {
        if (knob == KNOB_ALL || knob == i) {
            printf("knob %-3d", i + 1);
        }
    }
    printf("\n");
    for (int s = 0; s < SETTING_COUNT; s++) {
        printf("%-9s", SETTINGS[s].name);
        for (int i = 0; i < KNOB_COUNT; i++) {
            if (knob == KNOB_ALL || knob == i) {
                printf("%-8d", get_setting(i, SETTINGS[s].id));
            }
        }
        printf(" %s\n", SETTINGS[s].help);
    }
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* tune [knob]                -> show settings
 * tune [knob] <name> <value> -> change one setting (every knob if none given) */
static int cmd_tune(int argc, char **argv)
{
    int pos = 1;
    int k = take_knob(argc, argv, &pos);
    if (k == KNOB_BAD) {
        return 1;
    }
    if (pos == argc) {
        print_tuning(k);
        return 0;
    }
    if (argc - pos != 2) {
        printf("usage: tune [knob]                  (show settings)\n"
               "       tune [knob] <name> <value>   (no knob = change every knob)\n");
        return 1;
    }

    const setting_t *st = NULL;
    for (int s = 0; s < SETTING_COUNT; s++) {
        if (strcmp(argv[pos], SETTINGS[s].name) == 0) {
            st = &SETTINGS[s];
            break;
        }
    }
    if (st == NULL) {
        printf("unknown setting '%s' - type 'tune' to list them\n", argv[pos]);
        return 1;
    }
    int v = clampi(atoi(argv[pos + 1]), st->lo, st->hi);

    for (int i = 0; i < KNOB_COUNT; i++) {
        if (k == KNOB_ALL || k == i) {
            set_setting(i, st->id, v);
            zb_knob_save_settings(i);    /* kept across reboots and reflashes */
        }
    }
    print_tuning(k);
    return 0;
}

/* defaults [knob] */
static int cmd_defaults(int argc, char **argv)
{
    int pos = 1;
    int k = take_knob(argc, argv, &pos);
    if (k == KNOB_BAD) {
        return 1;
    }
    for (int i = 0; i < KNOB_COUNT; i++) {
        if (k == KNOB_ALL || k == i) {
            zb_knob_forget_settings(i);
        }
    }
    print_tuning(k);
    return 0;
}

/* ------------------------------------------------------------------ */

void console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "knob>";

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* Console on the chip's own USB port (the XIAO's only USB). This leaves
     * GPIO16/17 free for knobs. */
    esp_console_dev_usb_serial_jtag_config_t usb_cfg = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usb_cfg, &repl_cfg, &repl));
#else
    /* Console on UART0 (GPIO16/17) - the DevKit's port labelled UART. */
    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl));
#endif

    esp_console_register_help_command();

    const esp_console_cmd_t cmds[] = {
        { .command = "toggle", .help = "toggle [knob] - toggle that knob's bulbs (default knob 1)",
          .func = cmd_toggle },
        { .command = "on",     .help = "on [knob] - turn that knob's bulbs on", .func = cmd_on },
        { .command = "off",    .help = "off [knob] - turn that knob's bulbs off", .func = cmd_off },
        { .command = "step",   .help = "step [knob] <up|down> [size] - one brightness step",
          .func = cmd_step },
        { .command = "tune",   .help = "tune [knob] [name value] - show or change dimming settings "
                                       "live; no knob = all knobs",
          .func = cmd_tune },
        { .command = "defaults", .help = "defaults [knob] - forget saved tuning; no knob = all knobs",
          .func = cmd_defaults },
        { .command = "state",  .help = "state [knob] - what each knob thinks its lights are doing",
          .func = cmd_state },
        { .command = "stats",  .help = "stats [knob] - encoder and radio counters", .func = cmd_stats },
        { .command = "info",   .help = "Network status and which endpoint belongs to which knob",
          .func = cmd_info },
        { .command = "steer",  .help = "Look for a network to join now", .func = cmd_steer },
        { .command = "factoryreset", .help = "Leave the network, wipe Zigbee data, reboot",
          .func = cmd_reset },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
