/*
 * main.c - Zigbee rotary dimmer knobs (ESP32-C6). How many knobs, and on
 * which pins, is set by the KNOB_WIRING table in knob_config.h.
 */
#include "esp_log.h"
#include "driver/gpio.h"

#include "knob_config.h"
#include "input.h"
#include "zb_knob.h"

void console_start(void);

static const char *TAG = "main";

#ifdef BOARD_XIAO_ESP32C6
/* XIAO only: enable the RF switch, then select the external U.FL antenna. */
static void select_external_antenna(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << XIAO_GPIO_RF_SWITCH_EN) | (1ULL << XIAO_GPIO_ANT_SELECT),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(XIAO_GPIO_RF_SWITCH_EN, 0);
    gpio_set_level(XIAO_GPIO_ANT_SELECT, 1);
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Zigbee rotary dimmer knob starting");

#ifdef BOARD_XIAO_ESP32C6
    select_external_antenna();
#endif

    ESP_ERROR_CHECK(input_init());
    ESP_ERROR_CHECK(zb_knob_start());
    console_start();
}
