/*
 * zb_knob.c - Zigbee rotary dimmer knob (esp-zigbee-sdk 2.x).
 *
 * Written against Espressif's published SDK 2.x API reference. Every Zigbee
 * call below runs inside the Zigbee task: either in a stack callback, or in a
 * function posted to the Zigbee task queue. That is the SDK's rule for calling
 * the stack without taking its lock, so no locks are needed anywhere.
 *
 * How it works:
 *
 *   The knob keeps its own INTENT - a brightness number and on/off - and every
 *   command is derived from that. While you turn, it sends relative Step
 *   commands, which are fast and feel immediate. When you stop, it sends one
 *   absolute "go to this level", which is self-correcting: any bulb that
 *   missed a step is pulled back into line, so several bulbs stay together.
 *
 *   The button never sends Toggle. It sends an explicit On or Off, because a
 *   missed Toggle leaves bulbs permanently opposite each other.
 *
 *   Dimming never switches a bulb off; near the bottom it lands on minimum.
 *   Turning up from off comes on at minimum. The button restores the bulb's
 *   own previous brightness.
 *
 *   If you bind bulbs back to endpoint 2 in Z2M, their reports update the
 *   model so changes made in Home Assistant are picked up. Nothing depends on
 *   that: without it the knob simply trusts its own intent.
 */
#include "zb_knob.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_zigbee.h"   /* SDK 2.x all-in-one header */
#include "ezbee/zha.h"

#include "knob_config.h"
#include "input.h"

static const char *TAG = "knob";

/* ------------------------------------------------------------------ */
/* Settings                                                            */
/* ------------------------------------------------------------------ */

static const knob_tuning_t k_factory_tune = {
    .tick_ms        = KNOB_DEFAULT_TICK_MS,
    .units          = KNOB_DEFAULT_UNITS,
    .fade_ds        = KNOB_DEFAULT_FADE_DS,
    .settle_fade_ds = KNOB_DEFAULT_SETTLE_FADE_DS,
    .settle_ms      = KNOB_DEFAULT_SETTLE_MS,
    .floor_level    = KNOB_DEFAULT_FLOOR,
    .debounce_ms    = KNOB_DEFAULT_DEBOUNCE_MS,
    .log_tx         = (KNOB_DEFAULT_LOG_TX != 0),
    .rev_counts     = KNOB_DEFAULT_REV_COUNTS,
    .settle         = true,
};

knob_tuning_t g_knob_tune = {
    .tick_ms        = KNOB_DEFAULT_TICK_MS,
    .units          = KNOB_DEFAULT_UNITS,
    .fade_ds        = KNOB_DEFAULT_FADE_DS,
    .settle_fade_ds = KNOB_DEFAULT_SETTLE_FADE_DS,
    .settle_ms      = KNOB_DEFAULT_SETTLE_MS,
    .floor_level    = KNOB_DEFAULT_FLOOR,
    .debounce_ms    = KNOB_DEFAULT_DEBOUNCE_MS,
    .log_tx         = (KNOB_DEFAULT_LOG_TX != 0),
    .rev_counts     = KNOB_DEFAULT_REV_COUNTS,
    .settle         = true,
};

/* ------------------------------------------------------------------ */
/* Settings saved in flash, so tuning survives reboots and reflashes   */
/* ------------------------------------------------------------------ */

#define SETTINGS_NAMESPACE "knob"
#define SETTINGS_KEY       "tune"
#define SETTINGS_VERSION   1

typedef struct {
    uint8_t       version;
    bool          reverse;
    knob_tuning_t tune;
} knob_saved_t;

void zb_knob_save_settings(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not open settings storage: %s", esp_err_to_name(err));
        return;
    }
    knob_saved_t saved = {
        .version = SETTINGS_VERSION,
        .reverse = input_get_reverse(),
        .tune    = g_knob_tune,
    };
    err = nvs_set_blob(h, SETTINGS_KEY, &saved, sizeof(saved));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "settings not saved: %s", esp_err_to_name(err));
    }
}

void zb_knob_forget_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, SETTINGS_KEY);
        nvs_commit(h);
        nvs_close(h);
    }
    g_knob_tune = k_factory_tune;
    input_set_reverse(KNOB_ENCODER_REVERSE_DEFAULT != 0);
    ESP_LOGI(TAG, "settings back to the built-in defaults");
}

static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;                    /* nothing saved yet: keep the defaults */
    }
    knob_saved_t saved;
    size_t len = sizeof(saved);
    esp_err_t err = nvs_get_blob(h, SETTINGS_KEY, &saved, &len);
    nvs_close(h);

    /* Ignore anything saved by an older build with a different layout. */
    if (err == ESP_OK && len == sizeof(saved) && saved.version == SETTINGS_VERSION) {
        g_knob_tune = saved.tune;
        input_set_reverse(saved.reverse);
        ESP_LOGI(TAG, "settings restored from flash");
    }
}

#define LEVEL_MIN 1
#define LEVEL_MAX 254

/* ------------------------------------------------------------------ */
/* The model: what the knob believes the lights should be doing.       */
/* Every command is derived from this, so all bound bulbs converge on  */
/* the same numbers whatever they individually missed.                 */
/* ------------------------------------------------------------------ */

static volatile bool     s_on_pending;      /* waiting for a bulb to confirm on  */
static volatile uint32_t s_on_since_us;
static volatile uint8_t  s_wake_stage;      /* stubborn-bulb wake sequence      */
static esp_timer_handle_t s_wake_timer;

static uint8_t  s_level = 128;       /* intended brightness, 1..254       */
static bool     s_on    = true;      /* intended on/off                   */
static bool     s_known;             /* true once a bulb has confirmed it */

/* Encoder bookkeeping */
static volatile bool     s_stack_ready;
static volatile bool     s_eval_posted;
static volatile int32_t  s_consumed;
static volatile int32_t  s_carry;
static int8_t            s_dir;
static int32_t           s_rev_accum;
static uint32_t          s_last_accept_us;
static volatile uint32_t s_last_send_us;
static volatile uint32_t s_last_count_us;
static volatile bool     s_settle_pending;

/* Link quality: used to back off when the radio is struggling */
static volatile uint32_t s_tx_count, s_cnf_count, s_cnf_fail, s_tx_err;
static volatile uint16_t s_recent_tx, s_recent_fail;
static volatile uint16_t s_extra_tick_ms;
static uint32_t          s_last_backoff_us;
static uint32_t          s_last_warn_us;

/* Reports coming back from bulbs on endpoint 2 (optional) */
#define REPORT_SLOTS 8
static volatile uint8_t  s_rep_level[REPORT_SLOTS];
static volatile uint8_t  s_rep_level_n;
static volatile uint8_t  s_rep_on_yes, s_rep_on_no;
static volatile uint32_t s_rep_window_us;
static volatile bool     s_rep_window_open;
static volatile uint32_t s_reports_seen;

static esp_timer_handle_t s_poll_timer;
static esp_timer_handle_t s_retry_timer;
static volatile uint8_t   s_retry_mode;

static inline uint32_t now_us(void) { return (uint32_t)esp_timer_get_time(); }

static uint16_t effective_tick_ms(void)
{
    return (uint16_t)(g_knob_tune.tick_ms + s_extra_tick_ms);
}

static uint8_t clamp_level(int32_t v)
{
    if (v < LEVEL_MIN) return LEVEL_MIN;
    if (v > LEVEL_MAX) return LEVEL_MAX;
    return (uint8_t)v;
}

/* ------------------------------------------------------------------ */
/* Sending. Everything goes to the binding table - no addresses here.  */
/* ------------------------------------------------------------------ */

static void tx_confirm_cb(ezb_af_user_cnf_t *cnf, void *user_ctx)
{
    s_cnf_count++;
    if (s_recent_tx < 1000) {
        s_recent_tx++;
    }
    if (cnf != NULL && cnf->status != 0) {
        s_cnf_fail++;
        if (s_recent_fail < 1000) {
            s_recent_fail++;
        }
    }
}

static void fill_ctrl(ezb_zcl_cluster_cmd_ctrl_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->dst_addr.addr_mode = EZB_ADDR_MODE_NONE;   /* = use the bindings */
    ctrl->src_ep             = KNOB_EP;
    ctrl->dis_default_rsp    = true;
    ctrl->cnf_ctx.cb         = tx_confirm_cb;
    ctrl->cnf_ctx.user_ctx   = NULL;
}

static void note_tx(const char *what, ezb_err_t err)
{
    if (err == EZB_ERR_NONE) {
        s_tx_count++;
        s_last_send_us = now_us();
        return;
    }
    s_tx_err++;
    uint32_t now = now_us();
    if (now - s_last_warn_us > 2000000u) {
        s_last_warn_us = now;
        ESP_LOGW(TAG, "%s not sent (error %d) - is the knob bound to a bulb?", what, (int)err);
    }
}

static void send_onoff(bool on)
{
    ezb_zcl_on_off_cmd_t cmd;
    fill_ctrl(&cmd.cmd_ctrl);
    ezb_err_t err = on ? ezb_zcl_on_off_on_cmd_req(&cmd) : ezb_zcl_on_off_off_cmd_req(&cmd);
    note_tx(on ? "On" : "Off", err);
    if (err == EZB_ERR_NONE && g_knob_tune.log_tx) {
        ESP_LOGI(TAG, "TX %s", on ? "ON" : "OFF");
    }
}

/* Relative. Plain Step, NOT the with-on/off variant: dimming must never
 * switch a bulb off, or bulbs drop out one at a time near the bottom. */
static bool send_step(bool up, uint8_t size)
{
    ezb_zcl_level_step_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    fill_ctrl(&cmd.cmd_ctrl);
    cmd.payload.step_mode       = up ? 0 : 1;
    cmd.payload.step_size       = size;
    cmd.payload.transition_time = g_knob_tune.fade_ds;
    ezb_err_t err = ezb_zcl_level_step_cmd_req(&cmd);
    note_tx("Step", err);
    if (err == EZB_ERR_NONE && g_knob_tune.log_tx) {
        ESP_LOGI(TAG, "TX step %-4s %3u  -> model %3u", up ? "up" : "down", size, s_level);
    }
    return err == EZB_ERR_NONE;
}

/* Absolute, and therefore self-correcting: sending it twice changes nothing,
 * so any bulb that missed steps is pulled back into line. */
static bool send_level(uint8_t level, bool with_on_off, uint8_t fade_ds, const char *why)
{
    ezb_zcl_level_move_to_level_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    fill_ctrl(&cmd.cmd_ctrl);
    cmd.payload.level           = level;
    cmd.payload.transition_time = fade_ds;
    ezb_err_t err = with_on_off ? ezb_zcl_level_move_to_level_with_on_off_cmd_req(&cmd)
                                : ezb_zcl_level_move_to_level_cmd_req(&cmd);
    note_tx("Level", err);
    if (err == EZB_ERR_NONE && g_knob_tune.log_tx) {
        ESP_LOGI(TAG, "TX level %3u %s(%s)", level, with_on_off ? "+on " : "", why);
    }
    return err == EZB_ERR_NONE;
}

/* ------------------------------------------------------------------ */
/* Direction lock (contact chatter / the knob settling into a detent)  */
/* ------------------------------------------------------------------ */

static void filter_counts(int32_t raw, uint32_t now)
{
    if (s_dir != 0 && s_carry == 0 && (now - s_last_accept_us) > 500000u) {
        s_dir = 0;
        s_rev_accum = 0;
    }
    if (raw == 0) {
        return;
    }
    int8_t rdir = raw > 0 ? 1 : -1;
    if (s_dir == 0 || rdir == s_dir || g_knob_tune.rev_counts == 0) {
        s_dir = rdir;
        s_carry += raw;
        s_rev_accum = 0;
        s_last_accept_us = now;
        return;
    }
    s_rev_accum += raw;
    int32_t mag = s_rev_accum < 0 ? -s_rev_accum : s_rev_accum;
    if (mag >= g_knob_tune.rev_counts) {
        s_dir = rdir;
        s_carry += s_rev_accum;
        s_rev_accum = 0;
        s_last_accept_us = now;
    }
}

/* ------------------------------------------------------------------ */
/* Turning                                                             */
/* ------------------------------------------------------------------ */

/*
 * Coming on from off. Philips Hue bulbs ignore "go to level 1 and switch on",
 * so a single command isn't enough for a mixed set of bulbs. This sends three,
 * 200 ms apart: the combined command (which is all IKEA bulbs need), then a
 * plain On, then a plain "go to minimum" to undo the brightness that On
 * restores. Bulbs already at minimum see no change from the later two.
 */
static void wake_stage_in_zb(void *ctx)
{
    if (s_wake_stage == 1) {
        send_onoff(true);
    } else if (s_wake_stage == 2) {
        send_level(LEVEL_MIN, false, 0, "wake to min");
    }
    if (s_wake_stage < 2) {
        s_wake_stage++;
        esp_timer_start_once(s_wake_timer, 200000u);
    }
}

static void wake_timer_cb(void *arg)
{
    esp_zigbee_task_queue_post(wake_stage_in_zb, NULL);
}

static void start_wake_sequence(uint32_t now)
{
    s_level        = LEVEL_MIN;
    s_on           = true;
    s_on_pending   = true;       /* until a bulb reports back that it's on */
    s_on_since_us  = now;
    s_settle_pending = false;
    send_level(LEVEL_MIN, true, 0, "on from off");
    s_wake_stage = 1;
    esp_timer_stop(s_wake_timer);
    esp_timer_start_once(s_wake_timer, 200000u);
}

static void apply_turn(int32_t delta, uint32_t now)
{
    s_carry = 0;
    s_last_count_us = now;

    /* Off, and turned up: come on at minimum. Turning up from off feels like
     * it should light faintly, not jump back to whatever it was before - that
     * is what the button is for. */
    if (!s_on || s_on_pending) {
        if (delta > 0) {
            start_wake_sequence(now);
        }
        return;   /* turning down while off does nothing at all */
    }

    int32_t target = (int32_t)s_level + delta * (int32_t)g_knob_tune.units;
    uint8_t want = clamp_level(target);

    if (want == s_level) {
        return;   /* already against the top or bottom: send nothing */
    }

    /* Near the bottom, land exactly on minimum in one absolute command, so
     * every bulb ends up on the same value instead of straggling down. */
    if (want <= g_knob_tune.floor_level && target < (int32_t)s_level) {
        s_level = LEVEL_MIN;
        send_level(LEVEL_MIN, false, g_knob_tune.fade_ds, "floor");
        s_settle_pending = true;
        return;
    }

    int32_t change = (int32_t)want - (int32_t)s_level;
    uint8_t before = s_level;
    s_level = want;
    if (!send_step(change > 0, (uint8_t)(change > 0 ? change : -change))) {
        s_level = before;   /* never left the radio: don't let the model drift */
        return;
    }
    s_settle_pending = true;
}

/* The landing command, once your hand has stopped. */
static void settle_in_zb(void *ctx)
{
    s_settle_pending = false;
    if (!g_knob_tune.settle || !s_on) {
        return;
    }
    send_level(s_level, false, g_knob_tune.settle_fade_ds, "settle");
}

static void engine_eval_in_zb(void *ctx)
{
    s_eval_posted = false;

    const uint32_t now   = now_us();
    const int32_t  total = input_total();
    const int32_t  raw   = total - s_consumed;
    s_consumed = total;

    if (!ezb_bdb_dev_joined()) {
        s_carry = 0;
        return;
    }

    filter_counts(raw, now);
    if (s_carry != 0) {
        /* Counts only go out on the tick. The landing timer must never be
         * allowed to push a turn out early. */
        if ((now - s_last_send_us) >= (uint32_t)effective_tick_ms() * 1000u) {
            apply_turn(s_carry, now);
        }
        return;
    }

    if (s_settle_pending &&
        (now - s_last_count_us) >= (uint32_t)g_knob_tune.settle_ms * 1000u &&
        (now - s_last_send_us)  >= (uint32_t)g_knob_tune.settle_ms * 1000u) {
        settle_in_zb(NULL);
    }
}

static void button_press_in_zb(void *ctx)
{
    if (!ezb_bdb_dev_joined()) {
        ESP_LOGW(TAG, "button pressed but not on a network yet");
        return;
    }
    /* Never Toggle: with several bulbs, one missed toggle leaves them
     * permanently opposite. An explicit On or Off can only ever converge. */
    s_on = !s_on;
    s_on_pending = false;
    send_onoff(s_on);
    s_settle_pending = false;
}

/* ------------------------------------------------------------------ */
/* Reports from bulbs (optional, endpoint 2)                           */
/* ------------------------------------------------------------------ */

static uint8_t median_of(volatile uint8_t *v, uint8_t n)
{
    uint8_t tmp[REPORT_SLOTS];
    for (uint8_t i = 0; i < n; i++) {
        tmp[i] = v[i];
    }
    for (uint8_t i = 1; i < n; i++) {          /* insertion sort, n <= 8 */
        uint8_t x = tmp[i], j = i;
        while (j > 0 && tmp[j - 1] > x) { tmp[j] = tmp[j - 1]; j--; }
        tmp[j] = x;
    }
    return tmp[n / 2];
}

/* Only believe the bulbs when the knob has been still: during a turn they are
 * only echoing what we just sent, and mid-fade values would drag the model. */
static bool reports_welcome(uint32_t now)
{
    return (now - s_last_send_us) > 1500000u && (now - s_last_count_us) > 1500000u;
}

static void note_report_level(uint8_t level, uint32_t now)
{
    if (!reports_welcome(now)) {
        return;
    }
    if (!s_rep_window_open) {
        s_rep_window_open = true;
        s_rep_window_us = now;
        s_rep_level_n = 0;
        s_rep_on_yes = 0;
        s_rep_on_no = 0;
    }
    if (s_rep_level_n < REPORT_SLOTS) {
        s_rep_level[s_rep_level_n++] = level;
    }
    s_reports_seen++;
}

static void note_report_onoff(bool on, uint32_t now)
{
    s_reports_seen++;
    if (s_on_pending) {          /* a bulb answering settles the question */
        s_on_pending = false;
        s_on = on;
        s_known = true;
        return;
    }
    if (!reports_welcome(now)) {
        return;
    }
    if (!s_rep_window_open) {
        s_rep_window_open = true;
        s_rep_window_us = now;
        s_rep_level_n = 0;
        s_rep_on_yes = 0;
        s_rep_on_no = 0;
    }
    if (on) {
        s_rep_on_yes++;
    } else {
        s_rep_on_no++;
    }
}

/* Close the collection window: take the majority view, not an average, so a
 * single stray bulb can't drag the model. */
static void close_report_window_in_zb(void *ctx)
{
    s_rep_window_open = false;
    if (s_rep_level_n > 0) {
        uint8_t m = median_of(s_rep_level, s_rep_level_n);
        if (m >= LEVEL_MIN) {
            s_level = m;
            s_known = true;
        }
    }
    if (s_rep_on_yes != s_rep_on_no) {
        s_on = s_rep_on_yes > s_rep_on_no;
        s_known = true;
    }
    if (g_knob_tune.log_tx) {
        ESP_LOGI(TAG, "model updated from bulbs: level %u, %s", s_level, s_on ? "on" : "off");
    }
}

#if KNOB_USE_REPORTS
static void handle_report(void *message)
{
    if (message == NULL) {
        return;
    }
    const uint32_t now = now_us();
    ezb_zcl_cmd_report_attr_message_t *m = (ezb_zcl_cmd_report_attr_message_t *)message;
    const uint16_t cluster = m->info.cluster_id;

    for (const ezb_zcl_report_attr_variable_t *v = m->in.variables; v != NULL; v = v->next) {
        if (v->attr_value == NULL || v->attr_id != 0x0000) {
            continue;
        }
        if (cluster == EZB_ZCL_CLUSTER_ID_LEVEL) {
            note_report_level(*(const uint8_t *)v->attr_value, now);
        } else if (cluster == EZB_ZCL_CLUSTER_ID_ON_OFF) {
            note_report_onoff(*(const uint8_t *)v->attr_value != 0, now);
        }
    }
}

static void zcl_action_handler(ezb_zcl_core_action_callback_id_t id, void *message)
{
    if (id == EZB_ZCL_CORE_REPORT_ATTR_CB_ID) {
        handle_report(message);
    }
}
#endif /* KNOB_USE_REPORTS */

/* ------------------------------------------------------------------ */
/* Poller: decides when the Zigbee task has something to do            */
/* ------------------------------------------------------------------ */

static void update_backoff(uint32_t now)
{
    if (now - s_last_backoff_us < 3000000u) {
        return;
    }
    s_last_backoff_us = now;
    if (s_recent_tx >= 10) {
        /* More than a quarter of recent sends failing means the radio is
         * congested: slow down rather than making it worse. */
        if (s_recent_fail * 4 > s_recent_tx) {
            if (s_extra_tick_ms < 200) {
                s_extra_tick_ms = (uint16_t)(s_extra_tick_ms + 50);
                ESP_LOGW(TAG, "radio busy (%u of %u failed) - slowing to %u ms",
                         s_recent_fail, s_recent_tx, effective_tick_ms());
            }
        } else if (s_extra_tick_ms > 0) {
            s_extra_tick_ms = (uint16_t)(s_extra_tick_ms - 25);
        }
        s_recent_tx = 0;
        s_recent_fail = 0;
    }
}

static void poll_timer_cb(void *arg)
{
    uint8_t samples = (uint8_t)(g_knob_tune.debounce_ms / KNOB_POLL_MS);
    if (samples < 1) {
        samples = 1;
    }
    bool pressed = input_button_poll(samples);

    if (!s_stack_ready) {
        return;
    }
    if (pressed) {
        esp_zigbee_task_queue_post(button_press_in_zb, NULL);
    }

    const uint32_t now = now_us();
    update_backoff(now);

    if (s_rep_window_open && (now - s_rep_window_us) > 1000000u) {
        esp_zigbee_task_queue_post(close_report_window_in_zb, NULL);
    }

    if (s_eval_posted) {
        return;
    }

    const int32_t pending = (input_total() - s_consumed) + s_carry;
    bool due = false;

    if (pending != 0 && (now - s_last_send_us) >= (uint32_t)effective_tick_ms() * 1000u) {
        /* First move after a pause: gather the whole click (4 counts) before
         * sending, or give up waiting after a few ms. Still feels instant,
         * but sends one clean command instead of a small one then a big one. */
        if ((now - s_last_send_us) < 600000u) {
            due = true;
        } else {
            int32_t mag = pending < 0 ? -pending : pending;
            due = (mag >= 4) || ((now - input_last_edge_us()) >= 35000u);
        }
    }
    if (s_settle_pending &&
        (now - s_last_count_us) >= (uint32_t)g_knob_tune.settle_ms * 1000u &&
        (now - s_last_send_us)  >= (uint32_t)g_knob_tune.settle_ms * 1000u) {
        due = true;
    }

    if (due) {
        s_eval_posted = true;
        if (esp_zigbee_task_queue_post(engine_eval_in_zb, NULL) != ESP_OK) {
            s_eval_posted = false;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Data model: one HA Dimmer Switch endpoint                           */
/* ------------------------------------------------------------------ */

typedef ezb_zcl_cluster_desc_t (*cluster_create_fn)(const void *cfg, uint8_t role_mask);

/* Add a cluster only if the device template doesn't already include it. */
static void ensure_cluster(ezb_af_ep_desc_t ep, uint16_t cluster_id, uint8_t role,
                           cluster_create_fn create, const char *name)
{
    if (ezb_af_endpoint_get_cluster_desc(ep, cluster_id, role) != EZB_INVALID_ZCL_CLUSTER_DESC) {
        return;
    }
    ezb_err_t err = ezb_af_endpoint_add_cluster_desc(ep, create(NULL, role));
    ESP_LOGI(TAG, "added %s %s cluster: %s", name,
             role == EZB_ZCL_CLUSTER_CLIENT ? "client" : "server",
             err == EZB_ERR_NONE ? "ok" : "FAILED");
}

static void create_data_model(void)
{
    ezb_af_device_desc_t dev = ezb_af_create_device_desc();

    ezb_zha_dimmer_switch_config_t cfg = EZB_ZHA_DIMMER_SWITCH_CONFIG();
    cfg.basic_cfg.power_source = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;   /* USB/PoE */
    ezb_af_ep_desc_t ep = ezb_zha_create_dimmer_switch(KNOB_EP, &cfg);

    /* CLIENT clusters are what get bound to the bulb, and the SDK won't send a
     * command from an endpoint that lacks the matching client cluster. */
    ensure_cluster(ep, EZB_ZCL_CLUSTER_ID_ON_OFF,   EZB_ZCL_CLUSTER_CLIENT,
                   ezb_zcl_on_off_create_cluster_desc,   "On/Off");
    ensure_cluster(ep, EZB_ZCL_CLUSTER_ID_LEVEL,    EZB_ZCL_CLUSTER_CLIENT,
                   ezb_zcl_level_create_cluster_desc,    "Level Control");
    ensure_cluster(ep, EZB_ZCL_CLUSTER_ID_IDENTIFY, EZB_ZCL_CLUSTER_CLIENT,
                   ezb_zcl_identify_create_cluster_desc, "Identify");
    ensure_cluster(ep, EZB_ZCL_CLUSTER_ID_GROUPS,   EZB_ZCL_CLUSTER_CLIENT,
                   ezb_zcl_groups_create_cluster_desc,   "Groups");

    ezb_zcl_cluster_desc_t basic =
        ezb_af_endpoint_get_cluster_desc(ep, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    if (basic != EZB_INVALID_ZCL_CLUSTER_DESC) {
        ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                            KNOB_MANUFACTURER_NAME);
        ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                            KNOB_MODEL_IDENTIFIER);
    }

    ESP_ERROR_CHECK(esp_zigbee_err_to_esp(ezb_af_device_add_endpoint_desc(dev, ep)));

#if KNOB_USE_REPORTS
    /* Endpoint 2 exists only to receive reports. Bind each bulb's On/Off and
     * Level Control back to it in Z2M and the knob learns about changes made
     * elsewhere. Bulbs that can't or won't report simply never appear here,
     * and nothing else is affected. */
    ezb_zha_dimmer_switch_config_t rcfg = EZB_ZHA_DIMMER_SWITCH_CONFIG();
    ezb_af_ep_desc_t rep_ep = ezb_zha_create_dimmer_switch(KNOB_EP_REPORTS, &rcfg);
    ensure_cluster(rep_ep, EZB_ZCL_CLUSTER_ID_ON_OFF, EZB_ZCL_CLUSTER_CLIENT,
                   ezb_zcl_on_off_create_cluster_desc, "On/Off");
    ensure_cluster(rep_ep, EZB_ZCL_CLUSTER_ID_LEVEL, EZB_ZCL_CLUSTER_CLIENT,
                   ezb_zcl_level_create_cluster_desc, "Level Control");
    ESP_ERROR_CHECK(esp_zigbee_err_to_esp(ezb_af_device_add_endpoint_desc(dev, rep_ep)));
#endif
    ESP_ERROR_CHECK(esp_zigbee_err_to_esp(ezb_af_device_desc_register(dev)));
    ESP_LOGI(TAG, "endpoint %d registered: dimmer switch (On/Off + Level client)", KNOB_EP);
}

/* ------------------------------------------------------------------ */
/* Joining the network                                                 */
/* ------------------------------------------------------------------ */

static void print_info_in_zb(void *ctx)
{
    ezb_extaddr_t ext;
    ezb_get_extended_address(&ext);
    ESP_LOGI(TAG, "IEEE %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             ext.u8[7], ext.u8[6], ext.u8[5], ext.u8[4],
             ext.u8[3], ext.u8[2], ext.u8[1], ext.u8[0]);
    ESP_LOGI(TAG, "joined=%s  short=0x%04x  pan=0x%04x  channel=%u",
             ezb_bdb_dev_joined() ? "yes" : "no",
             (unsigned)ezb_get_short_address(), (unsigned)ezb_get_panid(),
             (unsigned)ezb_get_current_channel());
}

static void commission_in_zb(void *ctx)
{
    uint8_t mode = (uint8_t)(uintptr_t)ctx;
    ESP_LOGI(TAG, "%s (channel mask 0x%08lx) ...",
             mode == EZB_BDB_MODE_NETWORK_STEERING ? "looking for a network to join"
                                                   : "restoring network",
             (unsigned long)KNOB_ZB_CHANNEL_MASK);
    ezb_bdb_start_top_level_commissioning(mode);
}

static void retry_timer_cb(void *arg)
{
    esp_zigbee_task_queue_post(commission_in_zb, (void *)(uintptr_t)s_retry_mode);
}

static void schedule_commissioning(uint8_t mode, uint32_t delay_ms)
{
    s_retry_mode = mode;
    esp_timer_stop(s_retry_timer);   /* fine if it wasn't running */
    esp_timer_start_once(s_retry_timer, (uint64_t)delay_ms * 1000u);
}

static bool app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t type   = ezb_app_signal_get_type(app_signal);
    const void           *params = ezb_app_signal_get_params(app_signal);

    switch (type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        s_stack_ready = true;
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        return true;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        const ezb_bdb_signal_simple_params_t *p = params;
        if (p != NULL && p->status == EZB_BDB_STATUS_SUCCESS) {
            if (ezb_bdb_is_factory_new()) {
                commission_in_zb((void *)(uintptr_t)EZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "back on the network");
                print_info_in_zb(NULL);
            }
        } else {
            ESP_LOGW(TAG, "start-up failed (status %d), retrying in %d s",
                     p ? p->status : -1, KNOB_COMMISSION_RETRY_MS / 1000);
            schedule_commissioning(EZB_BDB_MODE_INITIALIZATION, KNOB_COMMISSION_RETRY_MS);
        }
        return true;
    }

    case EZB_BDB_SIGNAL_STEERING: {
        const ezb_bdb_signal_simple_params_t *p = params;
        if (p != NULL && p->status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "JOINED as a router - now bind endpoint %d to a bulb in Z2M", KNOB_EP);
            print_info_in_zb(NULL);
        } else {
            ESP_LOGW(TAG, "no network found (status %d) - is permit-join on in Z2M? "
                          "retrying in %d s", p ? p->status : -1, KNOB_COMMISSION_RETRY_MS / 1000);
            schedule_commissioning(EZB_BDB_MODE_NETWORK_STEERING, KNOB_COMMISSION_RETRY_MS);
        }
        return true;
    }

    case EZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "left the network - will look for one to join");
        schedule_commissioning(EZB_BDB_MODE_NETWORK_STEERING, KNOB_COMMISSION_RETRY_MS);
        return true;

    default:
        ESP_LOGD(TAG, "signal %s (0x%04x)", ezb_app_signal_to_string(type), (unsigned)type);
        return false;
    }
}

static void zb_task(void *arg)
{
    esp_zigbee_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.device_config.device_type                = EZB_NWK_DEVICE_TYPE_ROUTER;  /* mains powered */
    cfg.device_config.install_code_policy        = false;
    cfg.device_config.zczr_config.max_children   = KNOB_MAX_CHILDREN;
    cfg.platform_config.storage_partition_name   = "zb_storage";
    cfg.platform_config.radio_config.radio_mode  = ESP_ZIGBEE_RADIO_MODE_NATIVE;

    ESP_ERROR_CHECK(esp_zigbee_init(&cfg));
    ESP_ERROR_CHECK(esp_zigbee_err_to_esp(ezb_app_signal_add_handler(app_signal_handler)));
#if KNOB_USE_REPORTS
    ezb_zcl_core_action_handler_register(zcl_action_handler);
#endif

    create_data_model();

    /* Scans KNOB_ZB_CHANNEL_MASK (all standard channels by default, see
     * knob_config.h) while joining or rejoining. */
    ESP_ERROR_CHECK(esp_zigbee_err_to_esp(ezb_bdb_set_primary_channel_set(KNOB_ZB_CHANNEL_MASK)));
    ESP_ERROR_CHECK(esp_zigbee_err_to_esp(ezb_bdb_set_secondary_channel_set(KNOB_ZB_CHANNEL_MASK)));

    ESP_ERROR_CHECK(esp_zigbee_start(false));   /* we drive start-up via the signals */
    esp_zigbee_launch_mainloop();

    ESP_LOGE(TAG, "Zigbee main loop exited");
    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* Start-up                                                            */
/* ------------------------------------------------------------------ */

static esp_err_t init_nvs_partition(const char *label)
{
    esp_err_t err = label ? nvs_flash_init_partition(label) : nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition %s unreadable, erasing", label ? label : "nvs");
        err = label ? nvs_flash_erase_partition(label) : nvs_flash_erase();
        if (err == ESP_OK) {
            err = label ? nvs_flash_init_partition(label) : nvs_flash_init();
        }
    }
    return err;
}

esp_err_t zb_knob_start(void)
{
    ESP_ERROR_CHECK(init_nvs_partition(NULL));
    ESP_ERROR_CHECK(init_nvs_partition("zb_storage"));
    load_settings();

    const esp_timer_create_args_t retry_args = {
        .callback = retry_timer_cb, .name = "zb_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&retry_args, &s_retry_timer));

    s_consumed = input_total();

    const esp_timer_create_args_t wake_args = {
        .callback = wake_timer_cb, .name = "knob_wake",
    };
    ESP_ERROR_CHECK(esp_timer_create(&wake_args, &s_wake_timer));

    const esp_timer_create_args_t poll_args = {
        .callback = poll_timer_cb, .name = "knob_poll",
    };
    ESP_ERROR_CHECK(esp_timer_create(&poll_args, &s_poll_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_poll_timer, (uint64_t)KNOB_POLL_MS * 1000u));

    if (xTaskCreate(zb_task, "zigbee", 8192, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Console helpers                                                     */
/* ------------------------------------------------------------------ */

static esp_err_t post(esp_zigbee_callback_t cb, void *ctx)
{
    if (!s_stack_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_zigbee_task_queue_post(cb, ctx);
}

static void toggle_in_zb(void *ctx)  { button_press_in_zb(NULL); }
static void onoff_in_zb(void *ctx)
{
    s_on = (ctx != NULL);
    s_on_pending = false;
    send_onoff(s_on);
}
static void step_in_zb(void *ctx)
{
    uintptr_t v = (uintptr_t)ctx;
    uint8_t size = (uint8_t)(v & 0xFF);
    bool up = ((v >> 8) & 1) != 0;
    s_level = clamp_level((int32_t)s_level + (up ? size : -size));
    (void)send_step(up, size);
}
static void steer_in_zb(void *ctx)   { commission_in_zb((void *)(uintptr_t)EZB_BDB_MODE_NETWORK_STEERING); }
static void reset_in_zb(void *ctx)
{
    ESP_LOGW(TAG, "factory reset: leaving the network, wiping Zigbee data, rebooting");
    esp_zigbee_factory_reset();
}

esp_err_t zb_knob_toggle(void)          { return post(toggle_in_zb, NULL); }
esp_err_t zb_knob_onoff(bool on)        { return post(onoff_in_zb, on ? (void *)(uintptr_t)1 : NULL); }
esp_err_t zb_knob_step(bool up, uint8_t size)
{
    return post(step_in_zb, (void *)(uintptr_t)(((up ? 1u : 0u) << 8) | size));
}
esp_err_t zb_knob_print_info(void)      { return post(print_info_in_zb, NULL); }
esp_err_t zb_knob_steer(void)           { return post(steer_in_zb, NULL); }
esp_err_t zb_knob_factory_reset(void)   { return post(reset_in_zb, NULL); }

void zb_knob_print_stats(void)
{
    printf("encoder : count %" PRId32 ", rejected jumps %" PRIu32 ", direction %s, button %s\n",
           input_total(), input_rejected(), input_get_reverse() ? "reversed" : "normal",
           input_button_is_pressed() ? "held" : "released");
    printf("radio   : sent %" PRIu32 ", confirmed %" PRIu32 " (%" PRIu32 " failed), "
           "not sent %" PRIu32 ", gap %u ms\n",
           s_tx_count, s_cnf_count, s_cnf_fail, s_tx_err, effective_tick_ms());
    printf("reports : %" PRIu32 " received from bulbs%s\n", s_reports_seen,
           s_reports_seen ? "" : " (none - bind bulbs back to endpoint 2, or ignore)");
}

void zb_knob_print_state(void)
{
    printf("intended: %s, level %u of %u (%u%%)%s\n", s_on ? "ON" : "OFF", s_level, LEVEL_MAX,
           (unsigned)((s_level * 100u) / LEVEL_MAX), s_known ? ", confirmed by a bulb" : ", estimated");
    if (s_on_pending) {
        printf("          (waiting for a bulb to confirm it came on)\n");
    }
}
