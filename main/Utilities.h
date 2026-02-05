#ifndef UTILITIES_H
#define UTILITIES_H

#include "string.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_now.h"
#include "freertos/task.h"

static void mac_to_str(const uint8_t m[6], char *out, size_t n) {
    snprintf(out, n, "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static bool mac_equal(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a, b, 6) == 0;
}

static bool mac_is_zero(const uint8_t mac[6]) {
    for (int i = 0; i < 6; i++) {
        if (mac[i]) return false;
    }
    return true;
}

static int64_t now_us(void) {
    return esp_timer_get_time();
}

static esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel) {
    if (esp_now_is_peer_exist(mac)) return ESP_OK;
    esp_now_peer_info_t p = {0};
    memcpy(p.peer_addr, mac, 6);
    p.channel = channel;
    p.ifidx = ESP_IF_WIFI_STA;
    p.encrypt = false;
    esp_err_t r = esp_now_add_peer(&p);
    if (r != ESP_OK) ESP_LOGE(TAG, "add_peer %s", esp_err_to_name(r));
    return r;
}

static esp_err_t setup_broadcast_peer(uint8_t channel) {
    uint8_t b[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (esp_now_is_peer_exist(b)) esp_now_del_peer(b);
    esp_now_peer_info_t p = {0};
    memcpy(p.peer_addr, b, 6);
    p.channel = channel;
    p.ifidx = ESP_IF_WIFI_STA;
    p.encrypt = false;
    return esp_now_add_peer(&p);
}

static bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries) {
    for (int i = 0; i < retries; i++) {
        if (esp_now_send(mac, (uint8_t*)data, len) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(20));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(60 * (i + 1)));
    }
    return false;
}

static void restart_node(const char *reason) {
    ESP_LOGE(TAG, "Restarting: %s", reason ? reason : "no-reason");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

#endif // UTILITIES_H
