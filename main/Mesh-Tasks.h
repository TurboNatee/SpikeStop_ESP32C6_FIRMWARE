#ifndef MESH_TASKS_H
#define MESH_TASKS_H

#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"


void init_data_timer(void);
void init_alert_timer(void);
void initialize_sntp(void);
esp_err_t setup_broadcast_peer(uint8_t channel);
void led_root(void);
void led_isolated(void);
void set_led(uint8_t r, uint8_t g, uint8_t b);
int64_t now_us(void);
void restart_node(const char *reason);
bool mac_is_zero(const uint8_t mac[6]);
void mac_to_str(const uint8_t m[6], char *out, size_t n);

extern EventGroupHandle_t app_events;
extern const int EVT_WIFI_OK;
extern bool data_timer_running;
extern bool alert_timer_running;
extern node_role_t role;
extern uint8_t my_mac[6];
extern uint8_t root_mac[6];
extern uint8_t parent_mac[6];
extern uint8_t current_channel;
extern uint8_t current_layer;
extern volatile int64_t last_parent_seen_us;
extern volatile bool parent_link_up;
extern int8_t best_beacon_rssi;
extern QueueHandle_t influxdb_queue;


void root_beacon_task(void *arg) {
    uint8_t bmac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    setup_broadcast_peer(current_channel);
    while (1) {
        if (role != ROLE_ROOT) vTaskDelete(NULL);
        uint8_t pri = 0;
        wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        esp_wifi_get_channel(&pri, &sec);
        if (pri && pri != current_channel) {
            current_channel = pri;
            setup_broadcast_peer(current_channel);
        }
        beacon_pkt_t b = {0};
        b.hdr.type = PKT_BEACON;
        b.hdr.max_hops = 8;
        memcpy(b.root_mac, my_mac, 6);
        memcpy(b.parent_mac, my_mac, 6);
        b.channel = current_channel;
        b.layer = 0;
        esp_now_send(bmac, (uint8_t*)&b, sizeof(b));
        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
    }
}


void child_task(void *arg) {
    setup_broadcast_peer(current_channel);
    int scan_attempts = 0;
    int64_t last_root_attempt_us = 0;
    const int64_t ROOT_ATTEMPT_INTERVAL_US = 15000000; // 15s

    while (1) {
        int64_t n = now_us();

        // Parent timeout => restart fast
        if (parent_link_up && (n - last_parent_seen_us) > (int64_t)PARENT_LOSS_MS * 1000) {
            restart_node("Parent timeout");
        }
        if (!parent_link_up && !mac_is_zero(parent_mac) && (n - last_parent_seen_us) > (int64_t)PARENT_LOSS_MS * 2000) {
            restart_node("Stale parent state");
        }

        // Try to become root every 15s if isolated or link is down
        if ((role == ROLE_ISOLATED || !parent_link_up) && (n - last_root_attempt_us) > ROOT_ATTEMPT_INTERVAL_US) {
            last_root_attempt_us = n;
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(400));
            esp_wifi_connect();
            EventBits_t bits = xEventGroupWaitBits(app_events, EVT_WIFI_OK, pdFALSE, pdFALSE, pdMS_TO_TICKS(6000));
            if (bits & EVT_WIFI_OK) {
                wifi_ap_record_t ap;
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                    uint8_t new_ch = ap.primary;
                    role = ROLE_ROOT;
                    current_channel = new_ch;
                    memcpy(root_mac, my_mac, 6);
                    current_layer = 0;
                    parent_link_up = true;
                    memcpy(parent_mac, my_mac, 6);

                    xQueueReset(influxdb_queue);
                    for (int i = 0; i < 3; i++) {
                        set_led(0, 0, 255);
                        vTaskDelay(pdMS_TO_TICKS(160));
                        set_led(0, 0, 0);
                        vTaskDelay(pdMS_TO_TICKS(160));
                    }
                    led_root();

                    xTaskCreate(root_beacon_task, "root_beacon", 4096, NULL, 5, NULL);
                    if (!data_timer_running) init_data_timer();
                    if (!alert_timer_running) init_alert_timer();
                    initialize_sntp();

                    vTaskDelete(NULL);
                    return;
                }
            }
        }

        // Scan hopping while isolated
        if (role == ROLE_ISOLATED || !parent_link_up) {
            scan_attempts++;
            if (scan_attempts % 30 == 0) {
                uint8_t new_ch = current_channel + 1;
                if (new_ch > 13) new_ch = 1;
                current_channel = new_ch;
                esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
                vTaskDelay(pdMS_TO_TICKS(30));
                setup_broadcast_peer(current_channel);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#endif 
