/*
 * zb_knob.c - Zigbee rotary dimmer knobs (esp-zigbee-sdk 2.x).
 *
 * Written against Espressif's published SDK 2.x API reference. Every Zigbee
 * call below runs inside the Zigbee task: either in a stack callback, or in a
 * function posted to the Zigbee task queue. That is the SDK's rule for calling
 * the stack without taking its lock, so no locks are needed anywhere.
 *
 * How it works (the same for every knob, each completely on its own):
 *
 *   A knob keeps its own INTENT - a brightness number and on/off - and every
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
 *   If you bind bulbs back to a knob's report endpoint in Z2M, their reports
 *   update THAT knob's model so changes made in Home Assistant are picked up.
 *   Nothing depends on that: without it the knob simply trusts its own intent.
 *
 * How several knobs are kept apart:
 *
 *   Everything one knob knows lives in one knob_t (below), and there is an
 *   array of them - one per row of KNOB_WIRING. Every function that does
 *   knob work takes a `knob_t *k` - "which knob am I working on" - and only
 *   ever reads or writes k->something. Where the SDK or a timer calls us
 *   back, we hand it that same pointer as its "context" when we set it up,
 *   so it arrives back already knowing which knob it belongs to. There is no
 *   "current knob" variable anywhere, which is what makes crossover between
 *   knobs impossible by construction rather than by care.
 *
 *   What is deliberately SHARED: the radio, the network join/rejoin logic,
 *   flash set-up, and the single 10 ms poll timer. The poll timer holds no
 *   state of its own; it just visits each knob in turn, the same way one
 *   person can check four clocks.
 */
#include "zb_knob.h"

#include <stdio.h>
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

#define LEVEL_MIN 1
#define LEVEL_MAX 254

/* Reports coming back from bulbs: up to this many collected per window. */
#define REPORT_SLOTS 8

/* ------------------------------------------------------------------ */
/* One knob's complete state                                           */
/* ------------------------------------------------------------------ */

typedef struct {
    int      idx;                 /* 0-based position in KNOB_WIRING       */
    uint8_t  cmd_ep;              /* endpoint commands go out from         */
    uint8_t  report_ep;           /* endpoint reports arrive on (0 = none) */

    knob_tuning_t tune;           /* this knob's live settings             */

    /* The model: what this knob believes its lights should be doing.
     * Every command is derived from this, so all bound bulbs converge on
     * the same numbers whatever they individually missed. */
    uint8_t  level;               /* intended brightness, 1..254       */
    bool     on;                  /* intended on/off                   */
    bool     known;               /* true once a bulb has confirmed it */
    volatile bool     on_pending; /* waiting for a bulb to confirm on  */
    volatile uint32_t on_since_us;
    volatile uint8_t  wake_stage; /* stubborn-bulb wake sequence       */
    esp_timer_handle_t wake_timer;

    /* Encoder bookkeeping */
    volatile bool     eval_posted;
    volatile int32_t  consumed;
    volatile int32_t  carry;
    int8_t            dir;
    int32_t           rev_accum;
    uint32_t          last_accept_us;
    volatile uint32_t last_send_us;
    volatile uint32_t last_count_us;
    volatile bool     settle_pending;

    /* Link quality: used to back off when the radio is struggling */
    volatile uint32_t tx_count, cnf_count, cnf_fail, tx_err;
    volatile uint16_t recent_tx, recent_fail;
    volatile uint16_t extra_tick_ms;
    uint32_t          last_backoff_us;
    uint32_t          last_warn_us;

    /* Reports coming back from bulbs bound to report_ep (optional) */
    volatile uint8_t  rep_level[REPORT_SLOTS];
    volatile uint8_t  rep_level_n;
    volatile uint8_t  rep_on_yes, rep_on_no;
    volatile uint32_t rep_window_us;
    volatile bool     rep_window_open;
    volatile uint32_t reports_seen;
} knob_t;

static knob_t s_knobs[KNOB_COUNT];

/* Shared by all knobs: one radio, one network. */
static volatile bool      s_stack_ready;
static esp_timer_handle_t s_poll_timer;
static esp_timer_handle_t s_retry_timer;
static volatile uint8_t   s_retry_mode;

static inline uint32_t now_us(void) { return (uint32_t)esp_timer_get_time(); }

static uint16_t effective_tick_ms(const knob_t *k)
{
    return (uint16_t)(k->tune.tick_ms + k->extra_tick_ms);
}

static uint8_t clamp_level(int32_t v)
{
    if (v < LEVEL_MIN) return LEVEL_MIN;
    if (v > LEVEL_MAX) return LEVEL_MAX;
    return (uint8_t)v;
}

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

knob_tuning_t *zb_knob_tuning(int knob) { return &s_knobs[knob].tune; }

/* ------------------------------------------------------------------ */
/* Settings saved in flash, so tuning survives reboots and reflashes.  */
/* Each knob has its own entry: "tune1", "tune2", ...                  */
/* ------------------------------------------------------------------ */

#define SETTINGS_NAMESPACE   "knob"
#define SETTINGS_LEGACY_KEY  "tune"    /* what the single-knob build used */
#define SETTINGS_VERSION     1

typedef struct {
    uint8_t       version;
    bool          reverse;
    knob_tuning_t tune;
} knob_saved_t;

/* Fills `key` with this knob's flash key: "tune1" for the first knob etc.
 * (Flash keys can be at most 15 characters; this is always short.) */
#define SETTINGS_KEY_LEN 16
static void settings_key(const knob_t *k, char key[SETTINGS_KEY_LEN])
{
    snprintf(key, SETTINGS_KEY_LEN, "tune%d", k->idx + 1);
}

void zb_knob_save_settings(int knob)
{
    const knob_t *k = &s_knobs[knob];
    char key[SETTINGS_KEY_LEN];
    settings_key(k, key);

    nvs_handle_t h;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not open settings storage: %s", esp_err_to_name(err));
        return;
    }
    knob_saved_t saved = {
        .version = SETTINGS_VERSION,
        .reverse = input_get_reverse(knob),
        .tune    = k->tune,
    };
    err = nvs_set_blob(h, key, &saved, sizeof(saved));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "knob %d: settings not saved: %s", knob + 1, esp_err_to_name(err));
    }
}

void zb_knob_forget_settings(int knob)
{
    knob_t *k = &s_knobs[knob];
    char key[SETTINGS_KEY_LEN];
    settings_key(k, key);

    nvs_handle_t h;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, key);
        if (knob == 0) {
            nvs_erase_key(h, SETTINGS_LEGACY_KEY);   /* so it isn't picked up again */
        }
        nvs_commit(h);
        nvs_close(h);
    }
    k->tune = k_factory_tune;
    input_set_reverse(knob, KNOB_ENCODER_REVERSE_DEFAULT != 0);
    ESP_LOGI(TAG, "knob %d: settings back to the built-in defaults", knob + 1);
}

static bool read_saved(nvs_handle_t h, const char *key, knob_saved_t *out)
{
    size_t len = sizeof(*out);
    esp_err_t err = nvs_get_blob(h, key, out, &len);
    /* Ignore anything saved by an older build with a different layout. */
    return err == ESP_OK && len == sizeof(*out) && out->version == SETTINGS_VERSION;
}

static void load_settings(knob_t *k)
{
    nvs_handle_t h;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;                    /* nothing saved yet: keep the defaults */
    }
    char key[SETTINGS_KEY_LEN];
    settings_key(k, key);
    knob_saved_t saved;
    bool found = read_saved(h, key, &saved);

    /* Knob 1 falls back to what the single-knob firmware saved, so
     * upgrading doesn't lose tuning you'd already done. */
    if (!found && k->idx == 0) {
        found = read_saved(h, SETTINGS_LEGACY_KEY, &saved);
    }
    nvs_close(h);

    if (found) {
        k->tune = saved.tune;
        input_set_reverse(k->idx, saved.reverse);
        ESP_LOGI(TAG, "knob %d: settings restored from flash", k->idx + 1);
    }
}

/* ------------------------------------------------------------------ */
/* Sending. Everything goes to the binding table - no addresses here.  */
/* ------------------------------------------------------------------ */

/* Called by the stack when a send completes. `user_ctx` is the knob that
 * sent it (set in fill_ctrl), so each knob only counts its own sends. */
static void tx_confirm_cb(ezb_af_user_cnf_t *cnf, void *user_ctx)
{
    knob_t *k = (knob_t *)user_ctx;
    if (k == NULL) {
        return;
    }
    k->cnf_count++;
    if (k->recent_tx < 1000) {
        k->recent_tx++;
    }
    if (cnf != NULL && cnf->status != 0) {
        k->cnf_fail++;
        if (k->recent_fail < 1000) {
            k->recent_fail++;
        }
    }
}

/* Commands go out from THIS knob's endpoint, so the stack uses the bindings
 * made for that endpoint - i.e. this knob's bulbs and nobody else's. */
static void fill_ctrl(knob_t *k, ezb_zcl_cluster_cmd_ctrl_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->dst_addr.addr_mode = EZB_ADDR_MODE_NONE;   /* = use the bindings */
    ctrl->src_ep             = k->cmd_ep;
    ctrl->dis_default_rsp    = true;
    ctrl->cnf_ctx.cb         = tx_confirm_cb;
    ctrl->cnf_ctx.user_ctx   = k;
}

static void note_tx(knob_t *k, const char *what, ezb_err_t err)
{
    if (err == EZB_ERR_NONE) {
        k->tx_count++;
        k->last_send_us = now_us();
        return;
    }
    k->tx_err++;
    uint32_t now = now_us();
    if (now - k->last_warn_us > 2000000u) {
        k->last_warn_us = now;
        ESP_LOGW(TAG, "knob %d: %s not sent (error %d) - is endpoint %u bound to a bulb?",
                 k->idx + 1, what, (int)err, k->cmd_ep);
    }
}

static void send_onoff(knob_t *k, bool on)
{
    ezb_zcl_on_off_cmd_t cmd;
    fill_ctrl(k, &cmd.cmd_ctrl);
    ezb_err_t err = on ? ezb_zcl_on_off_on_cmd_req(&cmd) : ezb_zcl_on_off_off_cmd_req(&cmd);
    note_tx(k, on ? "On" : "Off", err);
    if (err == EZB_ERR_NONE && k->tune.log_tx) {
        ESP_LOGI(TAG, "knob %d: TX %s", k->idx + 1, on ? "ON" : "OFF");
    }
}

/* Relative. Plain Step, NOT the with-on/off variant: dimming must never
 * switch a bulb off, or bulbs drop out one at a time near the bottom. */
static bool send_step(knob_t *k, bool up, uint8_t size)
{
    ezb_zcl_level_step_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    fill_ctrl(k, &cmd.cmd_ctrl);
    cmd.payload.step_mode       = up ? 0 : 1;
    cmd.payload.step_size       = size;
    cmd.payload.transition_time = k->tune.fade_ds;
    ezb_err_t err = ezb_zcl_level_step_cmd_req(&cmd);
    note_tx(k, "Step", err);
    if (err == EZB_ERR_NONE && k->tune.log_tx) {
        ESP_LOGI(TAG, "knob %d: TX step %-4s %3u  -> model %3u",
                 k->idx + 1, up ? "up" : "down", size, k->level);
    }
    return err == EZB_ERR_NONE;
}

/* Absolute, and therefore self-correcting: sending it twice changes nothing,
 * so any bulb that missed steps is pulled back into line. */
static bool send_level(knob_t *k, uint8_t level, bool with_on_off, uint8_t fade_ds,
                       const char *why)
{
    ezb_zcl_level_move_to_level_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    fill_ctrl(k, &cmd.cmd_ctrl);
    cmd.payload.level           = level;
    cmd.payload.transition_time = fade_ds;
    ezb_err_t err = with_on_off ? ezb_zcl_level_move_to_level_with_on_off_cmd_req(&cmd)
                                : ezb_zcl_level_move_to_level_cmd_req(&cmd);
    note_tx(k, "Level", err);
    if (err == EZB_ERR_NONE && k->tune.log_tx) {
        ESP_LOGI(TAG, "knob %d: TX level %3u %s(%s)",
                 k->idx + 1, level, with_on_off ? "+on " : "", why);
    }
    return err == EZB_ERR_NONE;
}

/* ------------------------------------------------------------------ */
/* Direction lock (contact chatter / the knob settling into a detent)  */
/* ------------------------------------------------------------------ */

static void filter_counts(knob_t *k, int32_t raw, uint32_t now)
{
    if (k->dir != 0 && k->carry == 0 && (now - k->last_accept_us) > 500000u) {
        k->dir = 0;
        k->rev_accum = 0;
    }
    if (raw == 0) {
        return;
    }
    int8_t rdir = raw > 0 ? 1 : -1;
    if (k->dir == 0 || rdir == k->dir || k->tune.rev_counts == 0) {
        k->dir = rdir;
        k->carry += raw;
        k->rev_accum = 0;
        k->last_accept_us = now;
        return;
    }
    k->rev_accum += raw;
    int32_t mag = k->rev_accum < 0 ? -k->rev_accum : k->rev_accum;
    if (mag >= k->tune.rev_counts) {
        k->dir = rdir;
        k->carry += k->rev_accum;
        k->rev_accum = 0;
        k->last_accept_us = now;
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
 * Each knob has its own wake timer, so two knobs can wake at once.
 */
static void wake_stage_in_zb(void *ctx)
{
    knob_t *k = (knob_t *)ctx;
    if (k->wake_stage == 1) {
        send_onoff(k, true);
    } else if (k->wake_stage == 2) {
        send_level(k, LEVEL_MIN, false, 0, "wake to min");
    }
    if (k->wake_stage < 2) {
        k->wake_stage++;
        esp_timer_start_once(k->wake_timer, 200000u);
    }
}

static void wake_timer_cb(void *arg)
{
    esp_zigbee_task_queue_post(wake_stage_in_zb, arg);   /* arg = this knob */
}

static void start_wake_sequence(knob_t *k, uint32_t now)
{
    k->level          = LEVEL_MIN;
    k->on             = true;
    k->on_pending     = true;       /* until a bulb reports back that it's on */
    k->on_since_us    = now;
    k->settle_pending = false;
    send_level(k, LEVEL_MIN, true, 0, "on from off");
    k->wake_stage = 1;
    esp_timer_stop(k->wake_timer);
    esp_timer_start_once(k->wake_timer, 200000u);
}

static void apply_turn(knob_t *k, int32_t delta, uint32_t now)
{
    k->carry = 0;
    k->last_count_us = now;

    /* Off, and turned up: come on at minimum. Turning up from off feels like
     * it should light faintly, not jump back to whatever it was before - that
     * is what the button is for. */
    if (!k->on || k->on_pending) {
        if (delta > 0) {
            start_wake_sequence(k, now);
        }
        return;   /* turning down while off does nothing at all */
    }

    int32_t target = (int32_t)k->level + delta * (int32_t)k->tune.units;
    uint8_t want = clamp_level(target);

    if (want == k->level) {
        return;   /* already against the top or bottom: send nothing */
    }

    /* Near the bottom, land exactly on minimum in one absolute command, so
     * every bulb ends up on the same value instead of straggling down. */
    if (want <= k->tune.floor_level && target < (int32_t)k->level) {
        k->level = LEVEL_MIN;
        send_level(k, LEVEL_MIN, false, k->tune.fade_ds, "floor");
        k->settle_pending = true;
        return;
    }

    int32_t change = (int32_t)want - (int32_t)k->level;
    uint8_t before = k->level;
    k->level = want;
    if (!send_step(k, change > 0, (uint8_t)(change > 0 ? change : -change))) {
        k->level = before;   /* never left the radio: don't let the model drift */
        return;
    }
    k->settle_pending = true;
}

/* The landing command, once your hand has stopped. */
static void settle(knob_t *k)
{
    k->settle_pending = false;
    if (!k->tune.settle || !k->on) {
        return;
    }
    send_level(k, k->level, false, k->tune.settle_fade_ds, "settle");
}

static void engine_eval_in_zb(void *ctx)
{
    knob_t *k = (knob_t *)ctx;
    k->eval_posted = false;

    const uint32_t now   = now_us();
    const int32_t  total = input_total(k->idx);
    const int32_t  raw   = total - k->consumed;
    k->consumed = total;

    if (!ezb_bdb_dev_joined()) {
        k->carry = 0;
        return;
    }

    filter_counts(k, raw, now);
    if (k->carry != 0) {
        /* Counts only go out on the tick. The landing timer must never be
         * allowed to push a turn out early. */
        if ((now - k->last_send_us) >= (uint32_t)effective_tick_ms(k) * 1000u) {
            apply_turn(k, k->carry, now);
        }
        return;
    }

    if (k->settle_pending &&
        (now - k->last_count_us) >= (uint32_t)k->tune.settle_ms * 1000u &&
        (now - k->last_send_us)  >= (uint32_t)k->tune.settle_ms * 1000u) {
        settle(k);
    }
}

static void button_press_in_zb(void *ctx)
{
    knob_t *k = (knob_t *)ctx;
    if (!ezb_bdb_dev_joined()) {
        ESP_LOGW(TAG, "knob %d: button pressed but not on a network yet", k->idx + 1);
        return;
    }
    /* Never Toggle: with several bulbs, one missed toggle leaves them
     * permanently opposite. An explicit On or Off can only ever converge. */
    k->on = !k->on;
    k->on_pending = false;
    send_onoff(k, k->on);
    k->settle_pending = false;
}

/* ------------------------------------------------------------------ */
/* Reports from bulbs (optional, each knob's report endpoint)          */
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

#if KNOB_USE_REPORTS
/* Only believe the bulbs when THIS knob has been still: during a turn they
 * are only echoing what we just sent, and mid-fade values would drag the
 * model. Another knob turning doesn't matter - its bulbs report to its own
 * endpoint. */
static bool reports_welcome(const knob_t *k, uint32_t now)
{
    return (now - k->last_send_us) > 1500000u && (now - k->last_count_us) > 1500000u;
}

static void open_report_window(knob_t *k, uint32_t now)
{
    if (!k->rep_window_open) {
        k->rep_window_open = true;
        k->rep_window_us = now;
        k->rep_level_n = 0;
        k->rep_on_yes = 0;
        k->rep_on_no = 0;
    }
}

static void note_report_level(knob_t *k, uint8_t level, uint32_t now)
{
    if (!reports_welcome(k, now)) {
        return;
    }
    open_report_window(k, now);
    if (k->rep_level_n < REPORT_SLOTS) {
        k->rep_level[k->rep_level_n++] = level;
    }
    k->reports_seen++;
}

static void note_report_onoff(knob_t *k, bool on, uint32_t now)
{
    k->reports_seen++;
    if (k->on_pending) {          /* a bulb answering settles the question */
        k->on_pending = false;
        k->on = on;
        k->known = true;
        return;
    }
    if (!reports_welcome(k, now)) {
        return;
    }
    open_report_window(k, now);
    if (on) {
        k->rep_on_yes++;
    } else {
        k->rep_on_no++;
    }
}

#endif /* KNOB_USE_REPORTS */

/* Close the collection window: take the majority view, not an average, so a
 * single stray bulb can't drag the model. */
static void close_report_window_in_zb(void *ctx)
{
    knob_t *k = (knob_t *)ctx;
    if (!k->rep_window_open) {
        return;     /* already closed by an earlier post */
    }
    k->rep_window_open = false;
    if (k->rep_level_n > 0) {
        uint8_t m = median_of(k->rep_level, k->rep_level_n);
        if (m >= LEVEL_MIN) {
            k->level = m;
            k->known = true;
        }
    }
    if (k->rep_on_yes != k->rep_on_no) {
        k->on = k->rep_on_yes > k->rep_on_no;
        k->known = true;
    }
    if (k->tune.log_tx) {
        ESP_LOGI(TAG, "knob %d: model updated from bulbs: level %u, %s",
                 k->idx + 1, k->level, k->on ? "on" : "off");
    }
}

#if KNOB_USE_REPORTS
/* Which knob owns this endpoint? Reports bound to either of a knob's
 * endpoints count for that knob (the single-knob firmware accepted them on
 * both, so this keeps existing bindings working). NULL = no knob. */
static knob_t *knob_for_endpoint(uint8_t ep)
{
    for (int i = 0; i < KNOB_COUNT; i++) {
        knob_t *k = &s_knobs[i];
        if (ep == k->cmd_ep || (k->report_ep != 0 && ep == k->report_ep)) {
            return k;
        }
    }
    return NULL;
}

static void handle_report(void *message)
{
    if (message == NULL) {
        return;
    }
    const uint32_t now = now_us();
    ezb_zcl_cmd_report_attr_message_t *m = (ezb_zcl_cmd_report_attr_message_t *)message;
    const uint16_t cluster = m->info.cluster_id;

    /* The endpoint the report was addressed to says which knob it's for. */
    knob_t *k = knob_for_endpoint(m->info.dst_ep);
    if (k == NULL) {
        return;
    }

    for (const ezb_zcl_report_attr_variable_t *v = m->in.variables; v != NULL; v = v->next) {
        if (v->attr_value == NULL || v->attr_id != 0x0000) {
            continue;
        }
        if (cluster == EZB_ZCL_CLUSTER_ID_LEVEL) {
            note_report_level(k, *(const uint8_t *)v->attr_value, now);
        } else if (cluster == EZB_ZCL_CLUSTER_ID_ON_OFF) {
            note_report_onoff(k, *(const uint8_t *)v->attr_value != 0, now);
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

static void update_backoff(knob_t *k, uint32_t now)
{
    if (now - k->last_backoff_us < 3000000u) {
        return;
    }
    k->last_backoff_us = now;
    if (k->recent_tx >= 10) {
        /* More than a quarter of recent sends failing means the radio is
         * congested: slow down rather than making it worse. */
        if (k->recent_fail * 4 > k->recent_tx) {
            if (k->extra_tick_ms < 200) {
                k->extra_tick_ms = (uint16_t)(k->extra_tick_ms + 50);
                ESP_LOGW(TAG, "knob %d: radio busy (%u of %u failed) - slowing to %u ms",
                         k->idx + 1, k->recent_fail, k->recent_tx, effective_tick_ms(k));
            }
        } else if (k->extra_tick_ms > 0) {
            k->extra_tick_ms = (uint16_t)(k->extra_tick_ms - 25);
        }
        k->recent_tx = 0;
        k->recent_fail = 0;
    }
}

/* Everything the 10 ms poller does, for one knob. */
static void poll_one(knob_t *k, uint32_t now)
{
    uint8_t samples = (uint8_t)(k->tune.debounce_ms / KNOB_POLL_MS);
    if (samples < 1) {
        samples = 1;
    }
    bool pressed = input_button_poll(k->idx, samples);

    if (!s_stack_ready) {
        return;
    }
    if (pressed) {
        esp_zigbee_task_queue_post(button_press_in_zb, k);
    }

    update_backoff(k, now);

    if (k->rep_window_open && (now - k->rep_window_us) > 1000000u) {
        esp_zigbee_task_queue_post(close_report_window_in_zb, k);
    }

    if (k->eval_posted) {
        return;
    }

    const int32_t pending = (input_total(k->idx) - k->consumed) + k->carry;
    bool due = false;

    if (pending != 0 && (now - k->last_send_us) >= (uint32_t)effective_tick_ms(k) * 1000u) {
        /* First move after a pause: gather the whole click (4 counts) before
         * sending, or give up waiting after a few ms. Still feels instant,
         * but sends one clean command instead of a small one then a big one. */
        if ((now - k->last_send_us) < 600000u) {
            due = true;
        } else {
            int32_t mag = pending < 0 ? -pending : pending;
            due = (mag >= 4) || ((now - input_last_edge_us(k->idx)) >= 35000u);
        }
    }
    if (k->settle_pending &&
        (now - k->last_count_us) >= (uint32_t)k->tune.settle_ms * 1000u &&
        (now - k->last_send_us)  >= (uint32_t)k->tune.settle_ms * 1000u) {
        due = true;
    }

    if (due) {
        k->eval_posted = true;
        if (esp_zigbee_task_queue_post(engine_eval_in_zb, k) != ESP_OK) {
            k->eval_posted = false;
        }
    }
}

static void poll_timer_cb(void *arg)
{
    const uint32_t now = now_us();
    for (int i = 0; i < KNOB_COUNT; i++) {
        poll_one(&s_knobs[i], now);
    }
}

/* ------------------------------------------------------------------ */
/* Data model: one command endpoint (+ optional report endpoint) per   */
/* knob, all on one Zigbee device                                      */
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
    ESP_LOGD(TAG, "added %s %s cluster: %s", name,
             role == EZB_ZCL_CLUSTER_CLIENT ? "client" : "server",
             err == EZB_ERR_NONE ? "ok" : "FAILED");
    if (err != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "could not add %s cluster", name);
    }
}

static void add_command_endpoint(ezb_af_device_desc_t dev, uint8_t ep_id)
{
    ezb_zha_dimmer_switch_config_t cfg = EZB_ZHA_DIMMER_SWITCH_CONFIG();
    cfg.basic_cfg.power_source = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;   /* USB/PoE */
    ezb_af_ep_desc_t ep = ezb_zha_create_dimmer_switch(ep_id, &cfg);

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
}

#if KNOB_USE_REPORTS
/* A report endpoint exists only to receive reports. Bind each bulb's On/Off
 * and Level Control back to it in Z2M and its knob learns about changes made
 * elsewhere. Bulbs that can't or won't report simply never appear here, and
 * nothing else is affected. */
static void add_report_endpoint(ezb_af_device_desc_t dev, uint8_t ep_id)
{
    ezb_zha_dimmer_switch_config_t rcfg = EZB_ZHA_DIMMER_SWITCH_CONFIG();
    ezb_af_ep_desc_t rep_ep = ezb_zha_create_dimmer_switch(ep_id, &rcfg);
    ensure_cluster(rep_ep, EZB_ZCL_CLUSTER_ID_ON_OFF, EZB_ZCL_CLUSTER_CLIENT,
                   ezb_zcl_on_off_create_cluster_desc, "On/Off");
    ensure_cluster(rep_ep, EZB_ZCL_CLUSTER_ID_LEVEL, EZB_ZCL_CLUSTER_CLIENT,
                   ezb_zcl_level_create_cluster_desc, "Level Control");
    ESP_ERROR_CHECK(esp_zigbee_err_to_esp(ezb_af_device_add_endpoint_desc(dev, rep_ep)));
}
#endif

static void create_data_model(void)
{
    ezb_af_device_desc_t dev = ezb_af_create_device_desc();

    for (int i = 0; i < KNOB_COUNT; i++) {
        const knob_t *k = &s_knobs[i];
        add_command_endpoint(dev, k->cmd_ep);
#if KNOB_USE_REPORTS
        if (k->report_ep != 0) {
            add_report_endpoint(dev, k->report_ep);
        }
#endif
    }
    ESP_ERROR_CHECK(esp_zigbee_err_to_esp(ezb_af_device_desc_register(dev)));

    for (int i = 0; i < KNOB_COUNT; i++) {
        const knob_t *k = &s_knobs[i];
        if (k->report_ep != 0 && KNOB_USE_REPORTS) {
            ESP_LOGI(TAG, "knob %d: commands out on endpoint %u, reports in on endpoint %u",
                     i + 1, k->cmd_ep, k->report_ep);
        } else {
            ESP_LOGI(TAG, "knob %d: commands out on endpoint %u", i + 1, k->cmd_ep);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Joining the network (shared by all knobs)                           */
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
    for (int i = 0; i < KNOB_COUNT; i++) {
        const knob_t *k = &s_knobs[i];
        if (k->report_ep != 0 && KNOB_USE_REPORTS) {
            ESP_LOGI(TAG, "knob %d: bind endpoint %u to its bulbs (optional: bind the bulbs "
                          "back to endpoint %u)", i + 1, k->cmd_ep, k->report_ep);
        } else {
            ESP_LOGI(TAG, "knob %d: bind endpoint %u to its bulbs", i + 1, k->cmd_ep);
        }
    }
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
            ESP_LOGI(TAG, "JOINED as a router - now bind each knob's endpoint to its bulbs in Z2M");
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

/* Endpoints must be 1-240 (the Zigbee rule) and all different. */
static bool check_endpoints(void)
{
    bool ok = true;
    for (int i = 0; i < KNOB_COUNT; i++) {
        const uint8_t eps[2] = { KNOB_WIRING[i].cmd_ep, KNOB_WIRING[i].report_ep };
        for (int e = 0; e < 2; e++) {
            const uint8_t ep = eps[e];
            if (e == 1 && ep == 0) {
                continue;                       /* "no report endpoint" */
            }
            if (ep < 1 || ep > 240) {
                ESP_LOGE(TAG, "knob %d: endpoint %u is outside 1-240", i + 1, ep);
                ok = false;
            }
            for (int j = 0; j <= i; j++) {
                const uint8_t other[2] = { KNOB_WIRING[j].cmd_ep, KNOB_WIRING[j].report_ep };
                const int limit = (j == i) ? e : 2;
                for (int f = 0; f < limit; f++) {
                    if (other[f] != 0 && other[f] == ep) {
                        ESP_LOGE(TAG, "knob %d: endpoint %u is already used by knob %d",
                                 i + 1, ep, j + 1);
                        ok = false;
                    }
                }
            }
        }
    }
    return ok;
}

esp_err_t zb_knob_start(void)
{
    if (!check_endpoints()) {
        ESP_LOGE(TAG, "fix the endpoints in KNOB_WIRING (knob_config.h) and rebuild");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(init_nvs_partition(NULL));
    ESP_ERROR_CHECK(init_nvs_partition("zb_storage"));

    for (int i = 0; i < KNOB_COUNT; i++) {
        knob_t *k   = &s_knobs[i];
        k->idx       = i;
        k->cmd_ep    = KNOB_WIRING[i].cmd_ep;
        k->report_ep = KNOB_WIRING[i].report_ep;
        k->tune      = k_factory_tune;
        k->level     = 128;
        k->on        = true;
        load_settings(k);
        k->consumed  = input_total(i);

        /* Each knob gets its own wake timer. `arg` is the knob itself, so
         * the timer callback knows whose wake sequence to continue. */
        const esp_timer_create_args_t wake_args = {
            .callback = wake_timer_cb, .arg = k, .name = "knob_wake",
        };
        ESP_ERROR_CHECK(esp_timer_create(&wake_args, &k->wake_timer));
    }

    const esp_timer_create_args_t retry_args = {
        .callback = retry_timer_cb, .name = "zb_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&retry_args, &s_retry_timer));

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

/* The Zigbee task queue carries one pointer-sized value per job. For the
 * console commands we pack the knob number and the command's arguments
 * into that one number:  knob << 16 | flag << 8 | value. */
#define PACK_CTX(knob, flag, value) \
    ((void *)(uintptr_t)((((uintptr_t)(knob)) << 16) | (((uintptr_t)(flag) & 1u) << 8) | \
                         ((uintptr_t)(value) & 0xFFu)))
#define CTX_KNOB(ctx)  (&s_knobs[((uintptr_t)(ctx) >> 16) & 0xFFu])
#define CTX_FLAG(ctx)  ((((uintptr_t)(ctx)) >> 8) & 1u)
#define CTX_VALUE(ctx) ((uint8_t)((uintptr_t)(ctx) & 0xFFu))

static esp_err_t post(esp_zigbee_callback_t cb, void *ctx)
{
    if (!s_stack_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_zigbee_task_queue_post(cb, ctx);
}

static bool valid_knob(int knob) { return knob >= 0 && knob < KNOB_COUNT; }

static void onoff_in_zb(void *ctx)
{
    knob_t *k = CTX_KNOB(ctx);
    k->on = CTX_FLAG(ctx) != 0;
    k->on_pending = false;
    send_onoff(k, k->on);
}
static void step_in_zb(void *ctx)
{
    knob_t *k    = CTX_KNOB(ctx);
    bool    up   = CTX_FLAG(ctx) != 0;
    uint8_t size = CTX_VALUE(ctx);
    k->level = clamp_level((int32_t)k->level + (up ? size : -size));
    (void)send_step(k, up, size);
}
static void steer_in_zb(void *ctx)   { commission_in_zb((void *)(uintptr_t)EZB_BDB_MODE_NETWORK_STEERING); }
static void reset_in_zb(void *ctx)
{
    ESP_LOGW(TAG, "factory reset: leaving the network, wiping Zigbee data, rebooting");
    esp_zigbee_factory_reset();
}

esp_err_t zb_knob_toggle(int knob)
{
    if (!valid_knob(knob)) return ESP_ERR_INVALID_ARG;
    return post(button_press_in_zb, &s_knobs[knob]);
}
esp_err_t zb_knob_onoff(int knob, bool on)
{
    if (!valid_knob(knob)) return ESP_ERR_INVALID_ARG;
    return post(onoff_in_zb, PACK_CTX(knob, on, 0));
}
esp_err_t zb_knob_step(int knob, bool up, uint8_t size)
{
    if (!valid_knob(knob)) return ESP_ERR_INVALID_ARG;
    return post(step_in_zb, PACK_CTX(knob, up, size));
}
esp_err_t zb_knob_print_info(void)      { return post(print_info_in_zb, NULL); }
esp_err_t zb_knob_steer(void)           { return post(steer_in_zb, NULL); }
esp_err_t zb_knob_factory_reset(void)   { return post(reset_in_zb, NULL); }

void zb_knob_print_stats(int knob)
{
    const knob_t *k = &s_knobs[knob];
    printf("knob %d  (endpoint %u)\n", knob + 1, k->cmd_ep);
    printf("  encoder : count %" PRId32 ", rejected jumps %" PRIu32 ", direction %s, button %s\n",
           input_total(knob), input_rejected(knob), input_get_reverse(knob) ? "reversed" : "normal",
           input_button_is_pressed(knob) ? "held" : "released");
    printf("  radio   : sent %" PRIu32 ", confirmed %" PRIu32 " (%" PRIu32 " failed), "
           "not sent %" PRIu32 ", gap %u ms\n",
           k->tx_count, k->cnf_count, k->cnf_fail, k->tx_err, effective_tick_ms(k));
    if (k->report_ep != 0 && KNOB_USE_REPORTS) {
        printf("  reports : %" PRIu32 " received from bulbs%s\n", k->reports_seen,
               k->reports_seen ? "" : " (none - bind bulbs back to the report endpoint, or ignore)");
    }
}

void zb_knob_print_state(int knob)
{
    const knob_t *k = &s_knobs[knob];
    printf("knob %d: %s, level %u of %u (%u%%)%s\n", knob + 1, k->on ? "ON" : "OFF",
           k->level, LEVEL_MAX, (unsigned)((k->level * 100u) / LEVEL_MAX),
           k->known ? ", confirmed by a bulb" : ", estimated");
    if (k->on_pending) {
        printf("        (waiting for a bulb to confirm it came on)\n");
    }
}
