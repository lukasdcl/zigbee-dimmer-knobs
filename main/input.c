/*
 * input.c - rotary encoder + push button.
 */
#include "input.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "knob_config.h"

static const char *TAG = "input";

/*
 * State = (A << 1) | B, giving 0..3. Index = (old state << 2) | new state.
 *  +1 / -1 : one valid step in each direction
 *   0      : no change
 *   2      : impossible jump (both pins changed at once) - rejected
 */
static const int8_t STEP_TABLE[16] = {
     0,  1, -1,  2,
    -1,  0,  2,  1,
     1,  2,  0, -1,
     2, -1,  1,  0,
};

static volatile int32_t  s_total;
static volatile uint32_t s_rejected;
static volatile uint32_t s_last_edge_us;
static volatile uint8_t  s_state;
static volatile bool     s_reverse = (KNOB_ENCODER_REVERSE_DEFAULT != 0);

/* Button debounce state - only touched from the poll timer. */
static int     s_btn_stable    = 1;   /* 1 = released (pulled up) */
static int     s_btn_candidate = 1;
static uint8_t s_btn_count;

static inline uint8_t read_ab(void)
{
    return (uint8_t)((gpio_get_level(KNOB_GPIO_ENC_A) << 1) |
                      gpio_get_level(KNOB_GPIO_ENC_B));
}

/* Runs on every change of A or B. Keep it tiny. */
static void encoder_isr(void *arg)
{
    uint8_t now = read_ab();
    int8_t  d   = STEP_TABLE[(s_state << 2) | now];
    s_state = now;

    if (d == 2) {
        s_rejected++;
        return;
    }
    if (d != 0) {
        s_total += s_reverse ? -d : d;
        s_last_edge_us = (uint32_t)esp_timer_get_time();
    }
}

esp_err_t input_init(void)
{
    const gpio_pullup_t pull = KNOB_USE_INTERNAL_PULLUPS ? GPIO_PULLUP_ENABLE
                                                         : GPIO_PULLUP_DISABLE;

    gpio_config_t enc = {
        .pin_bit_mask = (1ULL << KNOB_GPIO_ENC_A) | (1ULL << KNOB_GPIO_ENC_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = pull,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&enc);
    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << KNOB_GPIO_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = pull,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&btn);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(5));   /* let the pull-ups settle */
    s_state = read_ab();

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {  /* already installed is fine */
        return err;
    }
    err = gpio_isr_handler_add(KNOB_GPIO_ENC_A, encoder_isr, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_isr_handler_add(KNOB_GPIO_ENC_B, encoder_isr, NULL);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "encoder A=GPIO%d B=GPIO%d, button GPIO%d, %s pull-ups; at rest A=%d B=%d SW=%d",
             KNOB_GPIO_ENC_A, KNOB_GPIO_ENC_B, KNOB_GPIO_BUTTON,
             KNOB_USE_INTERNAL_PULLUPS ? "built-in" : "external",
             gpio_get_level(KNOB_GPIO_ENC_A), gpio_get_level(KNOB_GPIO_ENC_B),
             gpio_get_level(KNOB_GPIO_BUTTON));
    return ESP_OK;
}

int32_t  input_total(void)          { return s_total; }
uint32_t input_rejected(void)       { return s_rejected; }
uint32_t input_last_edge_us(void)   { return s_last_edge_us; }
void     input_set_reverse(bool r)  { s_reverse = r; }
bool     input_get_reverse(void)    { return s_reverse; }
bool     input_button_is_pressed(void) { return s_btn_stable == 0; }

bool input_button_poll(uint8_t stable_samples_needed)
{
    int raw = gpio_get_level(KNOB_GPIO_BUTTON);

    if (raw == s_btn_candidate) {
        if (s_btn_count < 255) {
            s_btn_count++;
        }
    } else {
        s_btn_candidate = raw;
        s_btn_count = 1;
    }

    if (s_btn_count >= stable_samples_needed && s_btn_candidate != s_btn_stable) {
        s_btn_stable = s_btn_candidate;
        return s_btn_stable != 0;   /* fire on release, like a real switch */
    }
    return false;
}
