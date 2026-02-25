#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include "nvs_flash.h"
#include "esp_sntp.h"
#include "time.h"
#include "sys/time.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define ROOT_ALERT_DELTA_THRESHOLD 50.0f
#define ROOT_ALERT_WINDOW_SIZE 10

void send_data_packet(void);
float turbidity_read_voltage(void);
int turbidity_get_status(float v);
void queue_influxdb_data(const char *node_mac, int sensor_value, int temperature, int8_t rssi, int hops);
void mac_to_str(const uint8_t m[6], char *out, size_t n);
void set_led(uint8_t r, uint8_t g, uint8_t b);
void led_root(void);
void led_child(void);
void led_isolated(void);
void alert_led_effect(void);
extern bool turbidity_sensor_present;
extern uint8_t my_mac[6];
extern node_role_t role;

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
        static int root_values[ROOT_ALERT_WINDOW_SIZE] = {0};
        static int root_count = 0;
        static bool root_in_alert = false;

        float v = turbidity_read_voltage();
        if (!turbidity_sensor_present) {
            ESP_LOGW(TAG, "No turbidity - skip root autosend");
            return;
        }
        int st = turbidity_get_status(v);
        int sv = (int)(v * 100.0f);
        int t = 25;
        const char *state = (st == 0) ? "CLEAR" : (st == 1) ? "CLOUDY" : "DIRTY";
        char mac_str[18];
        mac_to_str(my_mac, mac_str, sizeof(mac_str));
        ESP_LOGI(TAG, "ROOT auto-send: %.2fV (%s) -> SENSOR:%d TEMP:%dC", v, state, sv, t);
        queue_influxdb_data(mac_str, sv, t, -65, 0);

        if (root_count < ROOT_ALERT_WINDOW_SIZE) {
            root_values[root_count++] = sv;
        } else {
            memmove(root_values,
                    root_values + 1,
                    sizeof(int) * (ROOT_ALERT_WINDOW_SIZE - 1));
            root_values[ROOT_ALERT_WINDOW_SIZE - 1] = sv;
        }

        float sum = 0.0f;
        for (int i = 0; i < root_count; i++) {
            sum += (float)root_values[i];
        }
        float avg = (root_count > 0) ? (sum / (float)root_count) : 0.0f;
        float max_delta = 0.0f;
        for (int i = 0; i < root_count; i++) {
            float d = fabsf((float)root_values[i] - avg);
            if (d > max_delta) {
                max_delta = d;
            }
        }

        if (max_delta > ROOT_ALERT_DELTA_THRESHOLD && !root_in_alert) {
            root_in_alert = true;
            alert_led_effect();
            ESP_LOGI(TAG, "Root self-alert: delta %.2f > %.2f", max_delta, ROOT_ALERT_DELTA_THRESHOLD);
        } else if (root_in_alert && max_delta <= ROOT_ALERT_DELTA_THRESHOLD) {
            root_in_alert = false;
            ESP_LOGI(TAG, "Root alert cleared: delta %.2f <= %.2f", max_delta, ROOT_ALERT_DELTA_THRESHOLD);
        }
        
        set_led(255, 255, 255);
        vTaskDelay(pdMS_TO_TICKS(40));
        led_root();
    } else {
        send_data_packet();
    }
}

#endif 
