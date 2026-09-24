/*
 * console_cmds.c - serial console on the DevKit's UART port.
 * Type `help` for the list. Everything here also works with Zigbee2MQTT and
 * Home Assistant switched off - commands go straight to the bound bulb.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_log.h"

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

static int cmd_toggle(int argc, char **argv) { return report(zb_knob_toggle()); }
static int cmd_on(int argc, char **argv)     { return report(zb_knob_onoff(true)); }
static int cmd_off(int argc, char **argv)    { return report(zb_knob_onoff(false)); }
static int cmd_info(int argc, char **argv)   { return report(zb_knob_print_info()); }
static int cmd_steer(int argc, char **argv)  { return report(zb_knob_steer()); }
static int cmd_reset(int argc, char **argv)  { return report(zb_knob_factory_reset()); }
static int cmd_stats(int argc, char **argv)  { zb_knob_print_stats(); return 0; }
static int cmd_state(int argc, char **argv)  { zb_knob_print_state(); return 0; }

/* step <up|down> [size] */
static int cmd_step(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: step <up|down> [size 1-254, default 16]\n");
        return 1;
    }
    bool up   = (argv[1][0] == 'u');
    int  size = (argc > 2) ? atoi(argv[2]) : 16;
    if (size < 1)   size = 1;
    if (size > 254) size = 254;
    return report(zb_knob_step(up, (uint8_t)size));
}

static void print_tuning(void)
{
    const knob_tuning_t *t = &g_knob_tune;
    printf("tick     %-6u  ms, minimum gap between dimming commands\n", t->tick_ms);
    printf("units    %-6u  brightness units per encoder count (4 counts per click here)\n", t->units);
    printf("fade     %-6u  tenths of a second each command fades over\n", t->fade_ds);
    printf("settle   %-6u  ms of stillness before the landing command\n", t->settle_ms);
    printf("sfade    %-6u  tenths of a second the landing command fades over\n", t->settle_fade_ds);
    printf("land     %-6d  1 = send the landing command that re-syncs the bulbs\n", t->settle ? 1 : 0);
    printf("floor    %-6u  below this level, go straight to minimum\n", t->floor_level);
    printf("debounce %-6u  ms the button must be stable\n", t->debounce_ms);
    printf("reverse  %-6d  1 = swap turning direction\n", input_get_reverse() ? 1 : 0);
    printf("rev      %-6u  opposite counts needed to change direction (4 per click); 0 = off\n", t->rev_counts);
    printf("log      %-6d  1 = print every command sent (turn off when judging feel)\n", t->log_tx ? 1 : 0);
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* tune                -> show everything
 * tune <name> <value> -> change one setting */
static int cmd_tune(int argc, char **argv)
{
    if (argc == 1) {
        print_tuning();
        return 0;
    }
    if (argc != 3) {
        printf("usage: tune            (show settings)\n"
               "       tune <name> <value>\n");
        return 1;
    }
    const char *k = argv[1];
    const char *s = argv[2];
    int v = atoi(s);
    knob_tuning_t *t = &g_knob_tune;

    if (strcmp(k, "tick") == 0)            { t->tick_ms        = (uint16_t)clampi(v, 20, 1000); }
    else if (strcmp(k, "units") == 0)      { t->units          = (uint8_t) clampi(v, 1, 64); }
    else if (strcmp(k, "fade") == 0)       { t->fade_ds        = (uint8_t) clampi(v, 0, 50); }
    else if (strcmp(k, "settle") == 0)     { t->settle_ms      = (uint16_t)clampi(v, 50, 2000); }
    else if (strcmp(k, "sfade") == 0)      { t->settle_fade_ds = (uint8_t) clampi(v, 0, 100); }
    else if (strcmp(k, "land") == 0)       { t->settle         = (v != 0); }
    else if (strcmp(k, "floor") == 0)      { t->floor_level    = (uint8_t) clampi(v, 1, 60); }
    else if (strcmp(k, "debounce") == 0)   { t->debounce_ms    = (uint16_t)clampi(v, 10, 200); }
    else if (strcmp(k, "reverse") == 0)    { input_set_reverse(v != 0); }
    else if (strcmp(k, "rev") == 0)        { t->rev_counts     = (uint8_t) clampi(v, 0, 40); }
    else if (strcmp(k, "log") == 0)        { t->log_tx         = (v != 0); }
    else {
        printf("unknown setting '%s' - type 'tune' to list them\n", k);
        return 1;
    }
    zb_knob_save_settings();    /* kept across reboots and reflashes */
    print_tuning();
    return 0;
}

static int cmd_defaults(int argc, char **argv)
{
    zb_knob_forget_settings();
    print_tuning();
    return 0;
}

void console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "knob>";
    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl));

    esp_console_register_help_command();

    const esp_console_cmd_t cmds[] = {
        { .command = "toggle", .help = "Toggle the bound bulb", .func = cmd_toggle },
        { .command = "on",     .help = "Turn the bound bulb on", .func = cmd_on },
        { .command = "off",    .help = "Turn the bound bulb off", .func = cmd_off },
        { .command = "step",   .help = "step <up|down> [size] - one brightness step", .func = cmd_step },
        { .command = "tune",   .help = "tune [name value] - show or change dimming settings live",
          .func = cmd_tune },
        { .command = "defaults", .help = "Forget saved tuning, back to built-in defaults",
          .func = cmd_defaults },
        { .command = "state",  .help = "What the knob thinks the lights are doing", .func = cmd_state },
        { .command = "stats",  .help = "Encoder and radio counters", .func = cmd_stats },
        { .command = "info",   .help = "Network status: address, PAN, channel", .func = cmd_info },
        { .command = "steer",  .help = "Look for a network to join now", .func = cmd_steer },
        { .command = "factoryreset", .help = "Leave the network, wipe Zigbee data, reboot",
          .func = cmd_reset },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
