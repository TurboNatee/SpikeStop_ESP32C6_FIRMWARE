#ifndef DATA_PACKETS_H
#define DATA_PACKETS_H

#include "string.h"
#include "stdio.h"

// Forward declarations
void queue_influxdb_data(const char *node_mac, int sensor_value, int temperature, int8_t rssi, int hops);
bool mac_is_zero(const uint8_t mac[6]);
esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel);
bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries);
void set_led(uint8_t r, uint8_t g, uint8_t b);
void led_child(void);
void led_root(void);
void led_isolated(void);
float turbidity_read_voltage(void);
int turbidity_get_status(float v);
void mac_to_str(const uint8_t m[6], char *out, size_t n);
extern bool turbidity_sensor_present;
extern uint8_t my_mac[6];
extern volatile bool parent_link_up;
extern uint8_t parent_mac[6];
extern uint8_t current_channel;
extern node_role_t role;

// ========= Packet Types =========
typedef enum {
    PKT_BEACON = 1,
    PKT_JOIN_REQUEST,
    PKT_JOIN_ACCEPT,
    PKT_DATA,
    PKT_ALERT_NOTIFY
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
    uint8_t src_mac[6];
    uint8_t payload[64];
} __attribute__((packed)) data_pkt_t;

typedef struct {
    mesh_hdr_t hdr;
    uint8_t target_mac[6];
    float delta;
    uint32_t timestamp;
} __attribute__((packed)) alert_notify_pkt_t;

void send_data_packet(void) {
    if (parent_link_up && !mac_is_zero(parent_mac)) {
        float v = turbidity_read_voltage();
        if (!turbidity_sensor_present) {
            ESP_LOGW(TAG, "No turbidity sensor — skip");
            return;
        }
        int status = turbidity_get_status(v);
        int sensor_value = (int)(v * 100.0f);
        int temp = 25;
        const char *state_str = (status == 0) ? "CLEAR" : (status == 1) ? "CLOUDY" : "DIRTY";

        data_pkt_t d = {0};
        d.hdr.type = PKT_DATA;
        d.hdr.max_hops = 8;
        memcpy(d.src_mac, my_mac, 6);
        snprintf((char*)d.payload, sizeof(d.payload), "Node:%02x%02x SENSOR:%d TEMP:%dC STATUS:%s",
                 my_mac[4], my_mac[5], sensor_value, temp, state_str);

        ensure_peer(parent_mac, current_channel);
        if (reliable_send(parent_mac, &d, sizeof(d), SEND_RETRY_LIMIT)) {
            set_led(255, 255, 255);
            vTaskDelay(pdMS_TO_TICKS(60));
            if (role == ROLE_CHILD) led_child();
            else if (role == ROLE_ROOT) led_root();
        }
    } else if (role == ROLE_ISOLATED) {
        for (int i = 0; i < 2; i++) {
            led_isolated();
            vTaskDelay(pdMS_TO_TICKS(100));
            set_led(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        led_isolated();
    }
}

void send_sample_data_packet(void) {
    char mac_str[18];
    mac_to_str(my_mac, mac_str, sizeof(mac_str));
    int sample_sensor_value = 250;
    int sample_temp = 22;
    queue_influxdb_data(mac_str, sample_sensor_value, sample_temp, -65, 0);

    if (parent_link_up && !mac_is_zero(parent_mac)) {
        data_pkt_t d = {0};
        d.hdr.type = PKT_DATA;
        d.hdr.max_hops = 8;
        memcpy(d.src_mac, my_mac, 6);
        snprintf((char*)d.payload, sizeof(d.payload), "Node:%02x%02x SENSOR:%d TEMP:%dC STATUS:%s SAMPLE",
                 my_mac[4], my_mac[5], sample_sensor_value, sample_temp, "CLEAR");
        ensure_peer(parent_mac, current_channel);
        reliable_send(parent_mac, &d, sizeof(d), SEND_RETRY_LIMIT);
    }
    set_led(255, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (role == ROLE_ROOT) led_root();
    else if (role == ROLE_CHILD) led_child();
    else led_isolated();
}

#endif 
