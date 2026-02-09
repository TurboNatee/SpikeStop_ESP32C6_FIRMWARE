#ifndef ALERT_HANDLER_H
#define ALERT_HANDLER_H

#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "string.h"
#include "stdlib.h"
#include "esp_timer.h"


esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel);
bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries);
void set_led(uint8_t r, uint8_t g, uint8_t b);
void led_root(void);

static bool parse_mac_string(const char* mac_str, uint8_t* mac_bytes) {
    int v[6];
    if (strchr(mac_str, ':')) {
        if (sscanf(mac_str, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
    } else if (strchr(mac_str, '-')) {
        if (sscanf(mac_str, "%x-%x-%x-%x-%x-%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
    } else {
        char hp[3] = {0};
        for (int i = 0; i < 6; i++) {
            if ((int)strlen(mac_str) < (i * 2 + 2)) return false;
            hp[0] = mac_str[i * 2];
            hp[1] = mac_str[i * 2 + 1];
            v[i] = strtol(hp, NULL, 16);
        }
    }
    for (int i = 0; i < 6; i++) {
        mac_bytes[i] = (uint8_t)v[i];
    }
    return true;
}

void send_alert_notification(const uint8_t target_mac[6], float delta) {
    alert_notify_pkt_t pkt = {0};
    pkt.hdr.type = PKT_ALERT_NOTIFY;
    pkt.hdr.max_hops = 8;
    pkt.hdr.hop_count = 0;
    memcpy(pkt.target_mac, target_mac, 6);
    pkt.delta = delta;
    pkt.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
    ensure_peer(target_mac, current_channel);
    if (reliable_send(target_mac, &pkt, sizeof(pkt), SEND_RETRY_LIMIT)) {
        set_led(255, 0, 255);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_root();
    }
}

static void process_alert_response(const char* response, int length) {
    char buf[4096];
    if (length > (int)sizeof(buf) - 1) length = sizeof(buf) - 1;
    memcpy(buf, response, length);
    buf[length] = '\0';

    static uint8_t alerted_nodes[5][6] = {{0}};
    static int alerted_count = 0;
    alerted_count = 0;
    int alerts_sent = 0;
    char *saveptr = NULL;
    char *line = strtok_r(buf, "\n", &saveptr);

    while (line && strstr(line, "node") == NULL) line = strtok_r(NULL, "\n", &saveptr);

    while ((line = strtok_r(NULL, "\n", &saveptr)) != NULL) {
        if (strlen(line) < 5) continue;
        char *fields[10];
        int fc = 0;
        char *t = strtok(line, ",");
        while (t && fc < 10) {
            fields[fc++] = t;
            t = strtok(NULL, ",");
        }
        if (fc >= 5) {
            char *node_mac = fields[4];
            uint8_t target[6];
            if (parse_mac_string(node_mac, target)) {
                bool seen = false;
                for (int i = 0; i < alerted_count; i++) {
                    if (!memcmp(alerted_nodes[i], target, 6)) {
                        seen = true;
                        break;
                    }
                }
                if (!seen) {
                    send_alert_notification(target, 150.0f);
                    alerts_sent++;
                    if (alerted_count < 5) {
                        memcpy(alerted_nodes[alerted_count], target, 6);
                        alerted_count++;
                    }
                }
            }
        }
    }
    if (alerts_sent > 0) ESP_LOGI(TAG, "Sent %d alert(s)", alerts_sent);
}

static esp_err_t poll_alerts_from_influxdb(void) {
    char url[512];
    snprintf(url, sizeof(url), "%s/api/v2/query?org=%s", INFLUXDB_URL, INFLUXDB_ORG);
    char query[512];
    snprintf(query, sizeof(query),
        "from(bucket:\"%s\") |> range(start: -10s) |> filter(fn: (r) => r._measurement == \"alert\" and r._field == \"active\" and r._value == true) |> last() |> keep(columns: [\"_time\", \"node\", \"_value\"]) |> yield()",
        ALERTS_DB);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .disable_auto_redirect = false,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        ESP_LOGE(TAG, "HTTP client failed");
        return ESP_FAIL;
    }

    char auth[300];
    snprintf(auth, sizeof(auth), "Token %s", ALERTS_INFLUXDB_TOKEN);
    esp_http_client_set_header(c, "Authorization", auth);
    esp_http_client_set_header(c, "Content-Type", "application/vnd.flux");
    esp_http_client_set_header(c, "Accept", "application/csv");
    esp_http_client_set_header(c, "Accept-Encoding", "identity");

    esp_err_t err = esp_http_client_open(c, strlen(query));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "open: %s", esp_err_to_name(err));
        esp_http_client_cleanup(c);
        return err;
    }
    int w = esp_http_client_write(c, query, strlen(query));
    if (w <= 0) {
        ESP_LOGE(TAG, "write failed");
        esp_http_client_cleanup(c);
        return ESP_FAIL;
    }

    esp_http_client_fetch_headers(c);
    int code = esp_http_client_get_status_code(c);
    char resp[4096] = {0};
    int total = 0, rd;
    while ((rd = esp_http_client_read(c, resp + total, sizeof(resp) - total - 1)) > 0) {
        total += rd;
    }
    resp[total] = '\0';

    if (code == 200 && total > 0) {
        if (strstr(resp, "true")) process_alert_response(resp, total);
    } else if (code != 200) {
        ESP_LOGE(TAG, "HTTP %d", code);
    }
    esp_http_client_cleanup(c);
    return ESP_OK;
}

#endif 
