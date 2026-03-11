#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include "nvs_flash.h"
#include "esp_sntp.h"
#include "time.h"
#include "sys/time.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef SPIKE_TURBIDITY_DELTA_CV
#define SPIKE_TURBIDITY_DELTA_CV 12
#endif

#ifndef SPIKE_TURBIDITY_MAJOR_DELTA_CV
#define SPIKE_TURBIDITY_MAJOR_DELTA_CV 30
#endif

#ifndef SPIKE_TURBIDITY_INSTANT_DELTA_CV
#define SPIKE_TURBIDITY_INSTANT_DELTA_CV 40
#endif

#ifndef SPIKE_TURBIDITY_CLEAR_CV
#define SPIKE_TURBIDITY_CLEAR_CV 6
#endif

#ifndef SPIKE_TEMP_DELTA_C
#define SPIKE_TEMP_DELTA_C 2
#endif

#ifndef SPIKE_TEMP_INSTANT_DELTA_C
#define SPIKE_TEMP_INSTANT_DELTA_C 4
#endif

#ifndef SPIKE_TEMP_CLEAR_C
#define SPIKE_TEMP_CLEAR_C 1
#endif

#ifndef SPIKE_IR_DELTA_MV
#define SPIKE_IR_DELTA_MV 10
#endif

#ifndef SPIKE_IR_INSTANT_DELTA_MV
#define SPIKE_IR_INSTANT_DELTA_MV 20
#endif

#ifndef SPIKE_IR_CLEAR_MV
#define SPIKE_IR_CLEAR_MV 5
#endif

#ifndef SPIKE_TRIGGER_CONFIRM_SAMPLES
#define SPIKE_TRIGGER_CONFIRM_SAMPLES 2
#endif

#ifndef SPIKE_CLEAR_CONFIRM_SAMPLES
#define SPIKE_CLEAR_CONFIRM_SAMPLES 2
#endif

void send_data_packet(void);
float turbidity_read_voltage(void);
int turbidity_get_status(float v);
float temperature_read_c(void);
float temperature_read_c_quiet(void);
float battery_read_voltage(void);
int battery_estimate_percent(float vbat);
bool ir_get_last_signal_mv(int *mv_out);
bool ir_get_last_broken(bool *broken_out);
void queue_influxdb_data(const char *node_mac,
                        int sensor_value,
                        int temperature,
                        int battery_cv,
                        int battery_pct,
                        int ir_signal_mv,
                        int ir_broken,
                        int8_t rssi,
                        int hops);
void mac_to_str(const uint8_t m[6], char *out, size_t n);
void set_led(uint8_t r, uint8_t g, uint8_t b);
void led_root(void);
void led_child(void);
void led_isolated(void);
void alert_led_effect(void);
extern bool turbidity_sensor_present;
extern uint8_t my_mac[6];
extern node_role_t role;

static bool root_have_baseline = false;
static int root_last_sensor = 0;
static int root_last_temp = 0;
static int root_last_ir_signal = -999;
static int root_last_ir_broken = -1;
static bool root_in_alert = false;

static void root_spike_evaluate(int sv, int t, int ir_signal_to_send, int ir_broken_to_send) {
    (void)ir_signal_to_send;
    (void)ir_broken_to_send;
    if (!root_have_baseline) {
        root_last_sensor = sv;
        root_last_temp = t;
        root_have_baseline = true;
        return;
    }

    int turb_delta = abs(sv - root_last_sensor);
    int temp_delta = abs(t - root_last_temp);

    bool trigger_now = (turb_delta >= SPIKE_TURBIDITY_INSTANT_DELTA_CV) ||
                       (temp_delta >= SPIKE_TEMP_INSTANT_DELTA_C);
    bool clear_now = (turb_delta <= SPIKE_TURBIDITY_CLEAR_CV) &&
                     (temp_delta <= SPIKE_TEMP_CLEAR_C);

    if (!root_in_alert && trigger_now) {
        root_in_alert = true;
        alert_led_effect();
        ESP_LOGW(TAG,
                 "Root spike alert: dTurb=%d dTemp=%d",
                 turb_delta,
                 temp_delta);
    } else if (root_in_alert && clear_now) {
        root_in_alert = false;
        ESP_LOGI(TAG,
                 "Root spike cleared: dTurb=%d dTemp=%d",
                 turb_delta,
                 temp_delta);
    }

    root_last_sensor = sv;
    root_last_temp = t;
}

static void process_fast_alert_check(void) {
    if (role != ROLE_ROOT || !turbidity_sensor_present) {
        return;
    }

    float v = turbidity_read_voltage();
    int sv = (int)(v * 100.0f);
    float temp_c = temperature_read_c_quiet();
    int t = isnan(temp_c) ? root_last_temp : (int)lroundf(temp_c);

    int ir_signal_mv = 0;
    bool ir_broken = true;
    int ir_signal_to_send = ir_get_last_signal_mv(&ir_signal_mv) ? ir_signal_mv : root_last_ir_signal;
    int ir_broken_to_send = ir_get_last_broken(&ir_broken) ? (ir_broken ? 1 : 0) : root_last_ir_broken;

    root_spike_evaluate(sv, t, ir_signal_to_send, ir_broken_to_send);
}

void initialize_sntp(void) {
    static bool sntp_started = false;
    if (sntp_started) {
        ESP_LOGW(TAG, "SNTP already running - skipping reinit");
        return;
    }

    ESP_LOGI(TAG, "SNTP init");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
#else
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
#endif

    sntp_started = true;

    time_t now = 0;
    struct tm ti = {0};
    int retry = 0;
    while (ti.tm_year < (2016 - 1900) && ++retry < 15) {
        vTaskDelay(pdMS_TO_TICKS(800));
        time(&now);
        localtime_r(&now, &ti);
    }

    if (ti.tm_year >= (2016 - 1900)) {
        ESP_LOGI(TAG, "SNTP time synchronized");
    } else {
        ESP_LOGW(TAG, "SNTP time sync timeout");
    }
}

void process_auto_send(void) {
    if (role == ROLE_ROOT) {
        float v = turbidity_read_voltage();
        if (!turbidity_sensor_present) {
            ESP_LOGW(TAG, "No turbidity - skip root autosend");
            return;
        }
        int st = turbidity_get_status(v);
        int sv = (int)(v * 100.0f);
        float temp_c = temperature_read_c();
        int t = isnan(temp_c) ? 0 : (int)lroundf(temp_c);
        const char *state = (st == 0) ? "CLEAR" : (st == 1) ? "CLOUDY" : "DIRTY";
        char mac_str[18];
        mac_to_str(my_mac, mac_str, sizeof(mac_str));
        if (isnan(temp_c)) {
            ESP_LOGW(TAG, "ROOT auto-send: %.2fV (%s) -> SENSOR:%d TEMP:read-failed", v, state, sv);
        } else {
            ESP_LOGI(TAG, "ROOT auto-send: %.2fV (%s) -> SENSOR:%d TEMP:%.2fC", v, state, sv, temp_c);
        }
        float vbat = battery_read_voltage();
        int battery_pct = battery_estimate_percent(vbat);
        if (!isnan(vbat) && battery_pct >= 0) {
            ESP_LOGI(TAG, "Battery: %.2fV (%d%%)", vbat, battery_pct);
        } else if (!isnan(vbat) && battery_pct < 0) {
            ESP_LOGW(TAG, "Battery: disconnected (%.2fV)", vbat);
        } else {
            ESP_LOGW(TAG, "Battery read failed");
        }

        int ir_signal_mv = 0;
        bool ir_broken = true;
        int ir_signal_to_send = ir_get_last_signal_mv(&ir_signal_mv) ? ir_signal_mv : -999;
        int ir_broken_to_send = ir_get_last_broken(&ir_broken) ? (ir_broken ? 1 : 0) : -1;

        int battery_cv = !isnan(vbat) ? (int)lroundf(vbat * 100.0f) : -1;
        int battery_pct_to_send = !isnan(vbat) ? battery_pct : -1;

        queue_influxdb_data(mac_str,
                            sv,
                            t,
                            battery_cv,
                            battery_pct_to_send,
                            ir_signal_to_send,
                            ir_broken_to_send,
                            -65,
                            0);
        root_spike_evaluate(sv, t, ir_signal_to_send, ir_broken_to_send);
        
    } else {
        send_data_packet();
    }
}

#endif 
