#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include "esp_now.h"
#include "esp_wifi.h"
#include <string.h>
#include <stdlib.h>
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

typedef struct {
    uint8_t mac[6];
    int last_sensor_value;
    int last_temperature;
    int last_ir_signal_mv;
    int last_ir_broken;
    bool have_baseline;
    bool in_alert;
    int trigger_confirm_count;
    int clear_confirm_count;
} node_alert_state_t;

static node_alert_state_t alert_states[10] = {0};
static int alert_state_count = 0;

void send_alert_notification(const uint8_t target_mac[6], float delta);
bool mac_equal(const uint8_t a[6], const uint8_t b[6]);
bool mac_is_zero(const uint8_t mac[6]);
void blink_orange(int times);
void queue_influxdb_data(const char *node_mac,
                        int sensor_value,
                        int temperature,
                        int battery_cv,
                        int battery_pct,
                        int ir_signal_mv,
                        int ir_broken,
                        int8_t rssi,
                        int hops);
void restart_node(const char *reason);
esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel);
bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries);
void led_child(void);
void led_root(void);
void provisioning_factory_reset(void);
extern volatile bool parent_link_up;
extern volatile int64_t last_parent_seen_us;
extern bool is_provisioned;
extern uint32_t provision_last_counter;

static uint32_t last_seen_prov_nonce = 0;
static int64_t last_prov_req_us = 0;
static uint32_t last_seen_reset_nonce = 0;

static void espnow_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    if (!mac) {
        return;
    }
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
    if (!info || !data || len < (int)sizeof(mesh_hdr_t)) {
        return;
    }
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
                    if (rssi > best_beacon_rssi) {
                        best_beacon_rssi = rssi;
                    }
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
        case PKT_PROV_OFFER: {
            if (len < (int)sizeof(prov_offer_pkt_t)) break;
            const prov_offer_pkt_t *po = (const prov_offer_pkt_t*)data;
            if (role == ROLE_ROOT || is_provisioned) {
                break;
            }
            if (strncmp(po->product_tag, PROV_PRODUCT_TAG, strlen(PROV_PRODUCT_TAG)) != 0) {
                break;
            }

            int64_t n = now_us();
            bool nonce_new = (po->nonce != last_seen_prov_nonce);
            bool retry_due = (n - last_prov_req_us) > 2000000;
            if (nonce_new || retry_due) {
                last_seen_prov_nonce = po->nonce;
                last_prov_req_us = n;

                ensure_peer(from, current_channel);
                prov_req_pkt_t rq = {0};
                rq.hdr.type = PKT_PROV_REQUEST;
                rq.hdr.max_hops = 1;
                memcpy(rq.child_mac, my_mac, 6);
                rq.nonce = po->nonce;
                reliable_send(from, &rq, sizeof(rq), SEND_RETRY_LIMIT);
                ESP_LOGI(TAG, "Sent provisioning request to root");
            }
            break;
        }
        case PKT_PROV_REQUEST: {
            if (len < (int)sizeof(prov_req_pkt_t)) break;
            const prov_req_pkt_t *rq = (const prov_req_pkt_t*)data;
            if (role != ROLE_ROOT || !provisioning_is_share_mode_active() || !is_provisioned) {
                break;
            }
            if (rq->nonce != provision_offer_nonce) {
                ESP_LOGW(TAG, "Provision request nonce mismatch");
                break;
            }

            ensure_peer(from, current_channel);

            prov_cred_pkt_t pc = {0};
            pc.hdr.type = PKT_PROV_CRED_BLOB;
            pc.hdr.max_hops = 1;
            memcpy(pc.target_mac, rq->child_mac, 6);
            pc.nonce = rq->nonce;
            pc.counter = ++provision_offer_nonce;
            pc.enc_flags = 0x01;

            prov_plain_payload_t plain = {0};
            plain.version = 1;
            strlcpy(plain.wifi_ssid, provision_cfg.wifi_ssid, sizeof(plain.wifi_ssid));
            strlcpy(plain.wifi_pass, provision_cfg.wifi_pass, sizeof(plain.wifi_pass));
            strlcpy(plain.venue_id, provision_cfg.venue_id, sizeof(plain.venue_id));
            strlcpy(plain.account_id, provision_cfg.account_id, sizeof(plain.account_id));

            uint16_t cipher_len = 0;

            if (provisioning_encrypt_payload(pc.nonce,
                                             pc.counter,
                                             my_mac,
                                             rq->child_mac,
                                             (const uint8_t *)&plain,
                                             sizeof(plain),
                                             pc.iv,
                                             pc.tag,
                                             pc.blob,
                                             sizeof(pc.blob),
                                             &cipher_len) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to encrypt provisioning payload");
                break;
            }
            pc.blob_len = cipher_len;

            reliable_send(from, &pc, sizeof(pc), SEND_RETRY_LIMIT);
            ESP_LOGI(TAG, "Encrypted provisioning blob sent");
            break;
        }
        case PKT_PROV_CRED_BLOB: {
            if (len < (int)sizeof(prov_cred_pkt_t)) break;
            const prov_cred_pkt_t *pc = (const prov_cred_pkt_t*)data;
            if (role == ROLE_ROOT || !mac_equal(pc->target_mac, my_mac)) {
                break;
            }
            if (pc->nonce != last_seen_prov_nonce) {
                ESP_LOGW(TAG, "Provision blob nonce mismatch");
                break;
            }
            if (pc->counter <= provision_last_counter) {
                ESP_LOGW(TAG, "Provision blob replay detected (counter=%lu, last=%lu)",
                         (unsigned long)pc->counter,
                         (unsigned long)provision_last_counter);
                break;
            }
            if (pc->blob_len == 0 || pc->blob_len > sizeof(pc->blob)) {
                ESP_LOGW(TAG, "Invalid provisioning blob length");
                break;
            }

            prov_plain_payload_t plain = {0};
            if (provisioning_decrypt_payload(pc->nonce,
                                             pc->counter,
                                             from,
                                             my_mac,
                                             pc->iv,
                                             pc->tag,
                                             pc->blob,
                                             pc->blob_len,
                                             (uint8_t *)&plain,
                                             sizeof(plain)) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to decrypt provisioning payload");
                break;
            }

            provisioning_config_t cfg = {0};
            cfg.version = plain.version;
            strlcpy(cfg.wifi_ssid, plain.wifi_ssid, sizeof(cfg.wifi_ssid));
            strlcpy(cfg.wifi_pass, plain.wifi_pass, sizeof(cfg.wifi_pass));
            strlcpy(cfg.venue_id, plain.venue_id, sizeof(cfg.venue_id));
            strlcpy(cfg.account_id, plain.account_id, sizeof(cfg.account_id));

            if (cfg.wifi_ssid[0]) {
                if (provisioning_save_to_nvs(&cfg) == ESP_OK) {
                    if (provisioning_save_counter_to_nvs(pc->counter) == ESP_OK) {
                        ESP_LOGI(TAG, "Provisioning saved securely; restarting to apply credentials");
                        restart_node("Provisioned over ESP-NOW (secure)");
                    }
                }
            } else {
                ESP_LOGW(TAG, "Invalid provisioning blob received");
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
                int sensor_value = 0;
                int temperature = 0;
                int battery_cv = -1;
                int battery_pct = -1;
                int ir_signal_mv = -999;
                int ir_broken = -1;

                bool parsed = false;
                if (sscanf((char*)dp->payload,
                           "N:%*2x%*2x S:%d T:%d V:%d P:%d I:%d B:%d",
                           &sensor_value,
                           &temperature,
                           &battery_cv,
                           &battery_pct,
                           &ir_signal_mv,
                           &ir_broken) == 6) {
                    parsed = true;
                } else if (sscanf((char*)dp->payload,
                                  "Node:%*s SENSOR:%d TEMP:%dC",
                                  &sensor_value,
                                  &temperature) == 2) {
                    parsed = true;
                }

                if (parsed) {
                    queue_influxdb_data(src,
                                        sensor_value,
                                        temperature,
                                        battery_cv,
                                        battery_pct,
                                        ir_signal_mv,
                                        ir_broken,
                                        dp->hdr.rssi,
                                        dp->hdr.hop_count);

                    node_alert_state_t *state = NULL;
                    for (int i = 0; i < alert_state_count; i++) {
                        if (mac_equal(alert_states[i].mac, dp->src_mac)) {
                            state = &alert_states[i];
                            break;
                        }
                    }

                    if (!state && alert_state_count < 10) {
                        state = &alert_states[alert_state_count++];
                        memcpy(state->mac, dp->src_mac, 6);
                        state->have_baseline = false;
                        state->in_alert = false;
                        state->trigger_confirm_count = 0;
                        state->clear_confirm_count = 0;
                    }

                    if (state) {
                        if (!state->have_baseline) {
                            state->last_sensor_value = sensor_value;
                            state->last_temperature = temperature;
                            state->last_ir_signal_mv = ir_signal_mv;
                            state->last_ir_broken = ir_broken;
                            state->have_baseline = true;
                        } else {
                            int turb_delta = abs(sensor_value - state->last_sensor_value);
                            int temp_delta = abs(temperature - state->last_temperature);

                            bool trigger_now = (turb_delta >= SPIKE_TURBIDITY_INSTANT_DELTA_CV) ||
                                               (temp_delta >= SPIKE_TEMP_INSTANT_DELTA_C);
                            bool clear_now = (turb_delta <= SPIKE_TURBIDITY_CLEAR_CV) &&
                                             (temp_delta <= SPIKE_TEMP_CLEAR_C);

                            float alert_delta = (float)turb_delta;
                            if (!state->in_alert && trigger_now) {
                                state->in_alert = true;
                                send_alert_notification(dp->src_mac, alert_delta);
                                ESP_LOGW(TAG,
                                         "Spike alert %s: dTurb=%d dTemp=%d",
                                         src,
                                         turb_delta,
                                         temp_delta);
                            } else if (state->in_alert && clear_now) {
                                state->in_alert = false;
                                ESP_LOGI(TAG,
                                         "Spike cleared %s: dTurb=%d dTemp=%d",
                                         src,
                                         turb_delta,
                                         temp_delta);
                            }

                            state->last_sensor_value = sensor_value;
                            state->last_temperature = temperature;
                            state->last_ir_signal_mv = ir_signal_mv;
                            state->last_ir_broken = ir_broken;
                        }
                    }
                }
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
        case PKT_REMOTE_RESET: {
            if (len < (int)sizeof(remote_reset_pkt_t)) break;
            const remote_reset_pkt_t *rr = (const remote_reset_pkt_t*)data;

            if (!mac_equal(rr->target_mac, my_mac)) {
                break;
            }

            if (rr->nonce == 0 || rr->nonce == last_seen_reset_nonce) {
                ESP_LOGW(TAG, "Remote reset ignored (replay/invalid nonce)");
                break;
            }

            last_seen_reset_nonce = rr->nonce;
            ESP_LOGW(TAG, "Remote factory reset received; executing");
            provisioning_factory_reset();
            break;
        }
        default: break;
    }
}

#endif 
