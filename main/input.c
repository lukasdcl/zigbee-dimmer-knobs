/*
 * input.c - rotary encoders + push buttons, one set per knob.
 *
 * Everything that used to be a loose `static` variable for "the" encoder now
 * lives inside an encoder_t, and there is one encoder_t per row of the
 * wiring table. Nothing is shared between knobs except the lookup table,
 * which never changes.
 */
#include "input.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "soc/uart_pins.h"
#include "sdkconfig.h"

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

/* Everything one encoder + button needs. `volatile` marks the fields the
 * interrupt writes, so the compiler always re-reads them. */
typedef struct {
    int               pin_a, pin_b, pin_btn;
    volatile int32_t  total;
    volatile uint32_t rejected;
    volatile uint32_t last_edge_us;
    volatile uint8_t  state;
    volatile bool     reverse;

    /* Button debounce state - only touched from the poll timer. */
    int               btn_stable;      /* 1 = released (pulled up) */
    int               btn_candidate;
    uint8_t           btn_count;
} encoder_t;

static encoder_t s_enc[KNOB_COUNT];

static inline uint8_t read_ab(const encoder_t *e)
{
    return (uint8_t)((gpio_get_level(e->pin_a) << 1) | gpio_get_level(e->pin_b));
}

/*
 * Runs on every change of A or B, for whichever knob it was. The GPIO driver
 * hands us back the pointer we registered for that pin (see input_init), so
 * one function serves every knob and each only ever touches its own state.
 * Keep it tiny.
 */
static void encoder_isr(void *arg)
{
    encoder_t *e  = (encoder_t *)arg;
    uint8_t   now = read_ab(e);
    int8_t    d   = STEP_TABLE[(e->state << 2) | now];
    e->state = now;

    if (d == 2) {
        e->rejected++;
        return;
    }
    if (d != 0) {
        e->total += e->reverse ? -d : d;
        e->last_edge_us = (uint32_t)esp_timer_get_time();
    }
}

/* ------------------------------------------------------------------ */
/* Checking the wiring table before touching any pins                  */
/* ------------------------------------------------------------------ */

/* Pins that must never be used for a knob, and why. */
static const char *pin_reserved_for(int pin)
{
    switch (pin) {
    case 8: case 9: case 15:
        return "a strapping pin (decides how the chip boots)";
    case 12: case 13:
        return "the USB port";
    case 24: case 25: case 26: case 27: case 28: case 29: case 30:
        return "the flash memory chip";
    default:
        break;
    }
#ifdef BOARD_XIAO_ESP32C6
    if (pin == XIAO_GPIO_RF_SWITCH_EN || pin == XIAO_GPIO_ANT_SELECT) {
        return "the XIAO's antenna switch";
    }
#endif
#if CONFIG_ESP_CONSOLE_UART_DEFAULT
    if (pin == U0TXD_GPIO_NUM || pin == U0RXD_GPIO_NUM) {
        return "the serial console (move the console to USB to free it)";
    }
#endif
    return NULL;
}

static bool check_wiring(void)
{
    bool ok = true;
    for (int k = 0; k < KNOB_COUNT; k++) {
        const int pins[3] = { KNOB_WIRING[k].enc_a, KNOB_WIRING[k].enc_b, KNOB_WIRING[k].button };
        for (int p = 0; p < 3; p++) {
            const int pin = pins[p];
            const char *why;
            if (!GPIO_IS_VALID_GPIO(pin)) {
                ESP_LOGE(TAG, "knob %d: GPIO%d doesn't exist on this chip", k + 1, pin);
                ok = false;
            } else if ((why = pin_reserved_for(pin)) != NULL) {
                ESP_LOGE(TAG, "knob %d: GPIO%d can't be used - it's %s", k + 1, pin, why);
                ok = false;
            }
            /* Same pin used twice, by this knob or an earlier one? */
            for (int k2 = 0; k2 <= k; k2++) {
                const int other[3] = { KNOB_WIRING[k2].enc_a, KNOB_WIRING[k2].enc_b,
                                       KNOB_WIRING[k2].button };
                const int limit = (k2 == k) ? p : 3;
                for (int p2 = 0; p2 < limit; p2++) {
                    if (other[p2] == pin) {
                        ESP_LOGE(TAG, "knob %d: GPIO%d is already used by knob %d",
                                 k + 1, pin, k2 + 1);
                        ok = false;
                    }
                }
            }
        }
    }
    return ok;
}

/* ------------------------------------------------------------------ */
/* Start-up                                                            */
/* ------------------------------------------------------------------ */

esp_err_t input_init(void)
{
    if (!check_wiring()) {
        ESP_LOGE(TAG, "fix KNOB_WIRING in knob_config.h and rebuild");
        return ESP_ERR_INVALID_ARG;
    }

    const gpio_pullup_t pull = KNOB_USE_INTERNAL_PULLUPS ? GPIO_PULLUP_ENABLE
                                                         : GPIO_PULLUP_DISABLE;

    /* Configure every knob's pins in one go: build up a mask with a bit set
     * for each pin, then hand the whole mask to gpio_config. */
    uint64_t enc_mask = 0, btn_mask = 0;
    for (int k = 0; k < KNOB_COUNT; k++) {
        enc_mask |= (1ULL << KNOB_WIRING[k].enc_a) | (1ULL << KNOB_WIRING[k].enc_b);
        btn_mask |= (1ULL << KNOB_WIRING[k].button);
    }

    gpio_config_t enc = {
        .pin_bit_mask = enc_mask,
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
        .pin_bit_mask = btn_mask,
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

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {  /* already installed is fine */
        return err;
    }

    for (int k = 0; k < KNOB_COUNT; k++) {
        encoder_t *e = &s_enc[k];
        e->pin_a         = KNOB_WIRING[k].enc_a;
        e->pin_b         = KNOB_WIRING[k].enc_b;
        e->pin_btn       = KNOB_WIRING[k].button;
        e->reverse       = (KNOB_ENCODER_REVERSE_DEFAULT != 0);
        e->btn_stable    = 1;
        e->btn_candidate = 1;
        e->state         = read_ab(e);

        /* The last argument is what the ISR receives as `arg`: this knob's
         * own state, so the ISR knows which knob moved. */
        err = gpio_isr_handler_add(e->pin_a, encoder_isr, e);
        if (err != ESP_OK) {
            return err;
        }
        err = gpio_isr_handler_add(e->pin_b, encoder_isr, e);
        if (err != ESP_OK) {
            return err;
        }

        ESP_LOGI(TAG, "knob %d: encoder A=GPIO%d B=GPIO%d, button GPIO%d, %s pull-ups; "
                      "at rest A=%d B=%d SW=%d",
                 k + 1, e->pin_a, e->pin_b, e->pin_btn,
                 KNOB_USE_INTERNAL_PULLUPS ? "built-in" : "external",
                 gpio_get_level(e->pin_a), gpio_get_level(e->pin_b),
                 gpio_get_level(e->pin_btn));
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Per-knob accessors                                                  */
/* ------------------------------------------------------------------ */

int32_t  input_total(int knob)                { return s_enc[knob].total; }
uint32_t input_rejected(int knob)             { return s_enc[knob].rejected; }
uint32_t input_last_edge_us(int knob)         { return s_enc[knob].last_edge_us; }
void     input_set_reverse(int knob, bool r)  { s_enc[knob].reverse = r; }
bool     input_get_reverse(int knob)          { return s_enc[knob].reverse; }
bool     input_button_is_pressed(int knob)    { return s_enc[knob].btn_stable == 0; }

bool input_button_poll(int knob, uint8_t stable_samples_needed)
{
    encoder_t *e = &s_enc[knob];
    int raw = gpio_get_level(e->pin_btn);

    if (raw == e->btn_candidate) {
        if (e->btn_count < 255) {
            e->btn_count++;
        }
    } else {
        e->btn_candidate = raw;
        e->btn_count = 1;
    }

    if (e->btn_count >= stable_samples_needed && e->btn_candidate != e->btn_stable) {
        e->btn_stable = e->btn_candidate;
        return e->btn_stable != 0;   /* fire on release, like a real switch */
    }
    return false;
}
