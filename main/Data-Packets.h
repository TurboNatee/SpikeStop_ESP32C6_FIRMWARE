#ifndef DATA_PACKETS_H
#define DATA_PACKETS_H

#include "string.h"
#include "stdio.h"
#include <math.h>

void queue_influxdb_data(const char *node_mac,
                        int sensor_value,
                        int temperature,
                        int battery_cv,
                        int battery_pct,
                        int ir_signal_mv,
                        int ir_signal_uv,
                        int ir_broken,
                        int8_t rssi,
                        int hops);
bool mac_is_zero(const uint8_t mac[6]);
esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel);
bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries);
void set_led(uint8_t r, uint8_t g, uint8_t b);
void led_child(void);
void led_root(void);
void led_isolated(void);
float turbidity_read_voltage(void);
int turbidity_get_status(float v);
float battery_read_voltage(void);
int battery_estimate_percent(float vbat);
bool ir_get_last_signal_mv(int *mv_out);
bool ir_get_last_signal_uv(int32_t *uv_out);
bool ir_get_last_broken(bool *broken_out);
void mac_to_str(const uint8_t m[6], char *out, size_t n);
extern bool turbidity_sensor_present;
extern uint8_t my_mac[6];
extern volatile bool parent_link_up;
extern uint8_t parent_mac[6];
extern uint8_t current_channel;
extern node_role_t role;

typedef enum {
    PKT_BEACON = 1,
    PKT_JOIN_REQUEST,
    PKT_JOIN_ACCEPT,
    PKT_PROV_OFFER,
    PKT_PROV_REQUEST,
    PKT_PROV_CRED_BLOB,
    PKT_DATA,
    PKT_ALERT_NOTIFY,
    PKT_REMOTE_RESET
} pkt_type_t;

typedef struct {
    uint8_t  type;
    uint8_t  hop_count;
    uint8_t  max_hops;
    int8_t   rssi;
    uint8_t  reserved;
} __attribute__((packed)) mesh_hdr_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t root_mac[6];
    uint8_t parent_mac[6];
    uint8_t channel;
    uint8_t layer;
} __attribute__((packed)) beacon_pkt_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t child_mac[6];
} __attribute__((packed)) join_req_pkt_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t parent_mac[6];
    uint8_t channel;
    uint8_t layer;
} __attribute__((packed)) join_acc_pkt_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t root_mac[6];
    uint32_t nonce;
    uint32_t ttl_ms;
    char product_tag[10];
} __attribute__((packed)) prov_offer_pkt_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t child_mac[6];
    uint32_t nonce;
} __attribute__((packed)) prov_req_pkt_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t target_mac[6];
    uint32_t nonce;
    uint32_t counter;
    uint8_t enc_flags;
    uint16_t blob_len;
    uint8_t iv[12];
    uint8_t tag[16];
    uint8_t blob[200];
} __attribute__((packed)) prov_cred_pkt_t;

typedef struct {
    uint32_t version;
    char wifi_ssid[33];
    char wifi_pass[65];
    char venue_id[40];
    char account_id[40];
} __attribute__((packed)) prov_plain_payload_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t src_mac[6];
    uint8_t payload[64];
} __attribute__((packed)) data_pkt_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t target_mac[6];
    float delta;
    uint32_t timestamp;
} __attribute__((packed)) alert_notify_pkt_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t target_mac[6];
    uint32_t nonce;
    uint32_t timestamp;
} __attribute__((packed)) remote_reset_pkt_t;

void send_data_packet(void) {
    if (!(parent_link_up && !mac_is_zero(parent_mac))) {
        if (role == ROLE_ISOLATED) {
            for (int i = 0; i < 2; i++) {
                led_isolated();
                vTaskDelay(pdMS_TO_TICKS(100));
                set_led(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            led_isolated();
        }
        return;
    }

    float v = turbidity_read_voltage();
    if (!turbidity_sensor_present) {
        ESP_LOGW(TAG, "No turbidity sensor - skip");
        return;
    }
    int sensor_value = (int)(v * 100.0f);
    float temp_c = temperature_read_c();
    int temp = 0;
    if (isnan(temp_c)) {
        ESP_LOGW(TAG, "Temperature read failed");
    } else {
        temp = (int)temp_c;
        ESP_LOGI(TAG, "Temperature: %.2f C", temp_c);
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
    int32_t ir_signal_uv = 0;
    bool ir_broken = true;
    int ir_signal_to_send = ir_get_last_signal_mv(&ir_signal_mv) ? ir_signal_mv : -999;
    int ir_signal_uv_to_send = ir_get_last_signal_uv(&ir_signal_uv) ? (int)ir_signal_uv : -999000;
    int ir_broken_to_send = ir_get_last_broken(&ir_broken) ? (ir_broken ? 1 : 0) : -1;
    int battery_cv = !isnan(vbat) ? (int)lroundf(vbat * 100.0f) : -1;
    int battery_pct_to_send = !isnan(vbat) ? battery_pct : -1;

    data_pkt_t d = {0};
    d.hdr.type = PKT_DATA;
    d.hdr.max_hops = 8;
    memcpy(d.src_mac, my_mac, 6);
    snprintf((char *)d.payload,
             sizeof(d.payload),
             "N:%02x%02x S:%d T:%d V:%d P:%d I:%d U:%d B:%d",
             my_mac[4],
             my_mac[5],
             sensor_value,
             temp,
             battery_cv,
             battery_pct_to_send,
             ir_signal_to_send,
             ir_signal_uv_to_send,
             ir_broken_to_send);

    ensure_peer(parent_mac, current_channel);
    if (reliable_send(parent_mac, &d, sizeof(d), SEND_RETRY_LIMIT)) {
        (void)0;
    }
}

#endif 
