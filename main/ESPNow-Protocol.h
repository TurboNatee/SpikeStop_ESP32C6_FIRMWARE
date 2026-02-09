#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include "esp_now.h"
#include "esp_wifi.h"

// Forward declarations
bool mac_equal(const uint8_t a[6], const uint8_t b[6]);
bool mac_is_zero(const uint8_t mac[6]);
void blink_orange(int times);
void queue_influxdb_data(const char *node_mac, int sensor_value, int temperature, int8_t rssi, int hops);
void restart_node(const char *reason);
esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel);
bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries);
void led_child(void);
void led_root(void);
extern volatile bool parent_link_up;
extern volatile int64_t last_parent_seen_us;

static void espnow_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    if (!mac) return;
    static int parent_fail_count = 0;
    if (status == ESP_NOW_SEND_SUCCESS) {
        if (!mac_equal(mac, (uint8_t[6]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF})) {
            parent_link_up = true;
            last_parent_seen_us = now_us();
            parent_fail_count = 0;
        }
    } else {
        if (!mac_is_zero(parent_mac) && mac_equal(mac, parent_mac)) {
            parent_fail_count++;
            if (parent_fail_count >= 5) {
                restart_node("Parent send failures");
            }
        }
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data || len < (int)sizeof(mesh_hdr_t)) return;
    int8_t rssi = info->rx_ctrl->rssi;
    const uint8_t *from = info->src_addr;
    mesh_hdr_t *hdr = (mesh_hdr_t*)data;
    hdr->rssi = rssi;

    switch (hdr->type) {
        case PKT_BEACON: {
            if (len < (int)sizeof(beacon_pkt_t)) break;
            const beacon_pkt_t *b = (const beacon_pkt_t*)data;

            if (role != ROLE_ROOT) {
                if (parent_link_up && mac_equal(from, parent_mac)) {
                    last_parent_seen_us = now_us();
                    if (rssi > best_beacon_rssi) best_beacon_rssi = rssi;
                    break;
                }
                bool better = (!parent_link_up) || (rssi > (best_beacon_rssi + 5));
                if (better) {
                    best_beacon_rssi = rssi;
                    memcpy(root_mac, b->root_mac, 6);
                    memcpy(parent_mac, from, 6);
                    current_channel = b->channel;

                    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    ensure_peer(parent_mac, current_channel);

                    join_req_pkt_t rq = {0};
                    rq.hdr.type = PKT_JOIN_REQUEST;
                    rq.hdr.max_hops = 5;
                    memcpy(rq.child_mac, my_mac, 6);
                    reliable_send(parent_mac, &rq, sizeof(rq), SEND_RETRY_LIMIT);
                }
            }
            break;
        }
        case PKT_JOIN_REQUEST: {
            if (len < (int)sizeof(join_req_pkt_t)) break;
            if (role == ROLE_ROOT || role == ROLE_CHILD) {
                ensure_peer(from, current_channel);
                join_acc_pkt_t ac = {0};
                ac.hdr.type = PKT_JOIN_ACCEPT;
                ac.hdr.max_hops = 5;
                memcpy(ac.parent_mac, my_mac, 6);
                ac.channel = current_channel;
                ac.layer = current_layer + 1;
                reliable_send(from, &ac, sizeof(ac), SEND_RETRY_LIMIT);
            }
            break;
        }
        case PKT_JOIN_ACCEPT: {
            if (role != ROLE_ROOT) {
                const join_acc_pkt_t *ac = (const join_acc_pkt_t*)data;
                if (role == ROLE_CHILD && mac_equal(parent_mac, ac->parent_mac)) {
                    last_parent_seen_us = now_us();
                    break;
                }
                memcpy(parent_mac, ac->parent_mac, 6);
                current_channel = ac->channel;
                current_layer = ac->layer;
                ensure_peer(parent_mac, current_channel);
                parent_link_up = true;
                last_parent_seen_us = now_us();
                role = ROLE_CHILD;
                led_child();
            }
            break;
        }
        case PKT_DATA: {
            if (len < (int)sizeof(data_pkt_t)) break;
            const data_pkt_t *dp = (const data_pkt_t*)data;
            last_parent_seen_us = now_us();

            if (role == ROLE_ROOT) {
                char src[18];
                mac_to_str(dp->src_mac, src, sizeof(src));
                int sensor_value = 0, temperature = 0;
                if (sscanf((char*)dp->payload, "Node:%*s SENSOR:%d TEMP:%dC", &sensor_value, &temperature) == 2) {
                    queue_influxdb_data(src, sensor_value, temperature, dp->hdr.rssi, dp->hdr.hop_count);
                }
                blink_orange(3);
            } else if (parent_link_up && dp->hdr.hop_count < dp->hdr.max_hops) {
                data_pkt_t fwd = *dp;
                fwd.hdr.hop_count++;
                reliable_send(parent_mac, &fwd, sizeof(fwd), SEND_RETRY_LIMIT);
            }
            break;
        }
        case PKT_ALERT_NOTIFY: {
            if (len < (int)sizeof(alert_notify_pkt_t)) break;
            const alert_notify_pkt_t *al = (const alert_notify_pkt_t*)data;
            if (mac_equal(al->target_mac, my_mac)) {
                alert_led_effect();
                if (parent_link_up && al->hdr.hop_count < al->hdr.max_hops) {
                    alert_notify_pkt_t fwd = *al;
                    fwd.hdr.hop_count++;
                    reliable_send(parent_mac, &fwd, sizeof(fwd), SEND_RETRY_LIMIT);
                }
            }
            break;
        }
        default: break;
    }
}

#endif 
