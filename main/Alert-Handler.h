#ifndef ALERT_HANDLER_H
#define ALERT_HANDLER_H

#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "string.h"
#include "stdlib.h"
#include "esp_timer.h"

extern uint8_t my_mac[6];

void alert_led_effect(void);
bool mac_equal(const uint8_t a[6], const uint8_t b[6]);
esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel);
bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries);
void set_led(uint8_t r, uint8_t g, uint8_t b);
void led_root(void);
void provisioning_factory_reset(void);
void provisioning_toggle_share_mode(void);

#define ALERT_HTTP_BUF_SIZE 4096

static char alert_http_resp_buf[ALERT_HTTP_BUF_SIZE];
static char reset_http_resp_buf[ALERT_HTTP_BUF_SIZE];
static char share_http_resp_buf[ALERT_HTTP_BUF_SIZE];

typedef struct {
    uint8_t mac[6];
    int64_t last_issued_us;
} reset_dedupe_entry_t;

typedef struct {
    uint8_t mac[6];
    int64_t last_issued_us;
} share_dedupe_entry_t;

static reset_dedupe_entry_t reset_dedupe[8] = {0};
static share_dedupe_entry_t share_dedupe[8] = {0};

static bool reset_recently_issued(const uint8_t target_mac[6]) {
    int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < 8; i++) {
        if (mac_equal(reset_dedupe[i].mac, target_mac)) {
            int64_t elapsed_ms = (now_us - reset_dedupe[i].last_issued_us) / 1000;
            return elapsed_ms >= 0 && elapsed_ms < REMOTE_RESET_COOLDOWN_MS;
        }
    }
    return false;
}

static void mark_reset_issued(const uint8_t target_mac[6]) {
    int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < 8; i++) {
        if (mac_equal(reset_dedupe[i].mac, target_mac) || reset_dedupe[i].last_issued_us == 0) {
            memcpy(reset_dedupe[i].mac, target_mac, 6);
            reset_dedupe[i].last_issued_us = now_us;
            return;
        }
    }

    int oldest_idx = 0;
    for (int i = 1; i < 8; i++) {
        if (reset_dedupe[i].last_issued_us < reset_dedupe[oldest_idx].last_issued_us) {
            oldest_idx = i;
        }
    }
    memcpy(reset_dedupe[oldest_idx].mac, target_mac, 6);
    reset_dedupe[oldest_idx].last_issued_us = now_us;
}

static bool share_recently_issued(const uint8_t target_mac[6]) {
    int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < 8; i++) {
        if (mac_equal(share_dedupe[i].mac, target_mac)) {
            int64_t elapsed_ms = (now_us - share_dedupe[i].last_issued_us) / 1000;
            return elapsed_ms >= 0 && elapsed_ms < REMOTE_SHARE_COOLDOWN_MS;
        }
    }
    return false;
}

static void mark_share_issued(const uint8_t target_mac[6]) {
    int64_t now_us = esp_timer_get_time();
    for (int i = 0; i < 8; i++) {
        if (mac_equal(share_dedupe[i].mac, target_mac) || share_dedupe[i].last_issued_us == 0) {
            memcpy(share_dedupe[i].mac, target_mac, 6);
            share_dedupe[i].last_issued_us = now_us;
            return;
        }
    }

    int oldest_idx = 0;
    for (int i = 1; i < 8; i++) {
        if (share_dedupe[i].last_issued_us < share_dedupe[oldest_idx].last_issued_us) {
            oldest_idx = i;
        }
    }
    memcpy(share_dedupe[oldest_idx].mac, target_mac, 6);
    share_dedupe[oldest_idx].last_issued_us = now_us;
}

static bool parse_mac_string(const char *mac_str, uint8_t *mac_bytes) {
    int v[6];
    if (strchr(mac_str, ':')) {
        if (sscanf(mac_str, "%x:%x:%x:%x:%x:%x",
                   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
            return false;
        }
    } else if (strchr(mac_str, '-')) {
        if (sscanf(mac_str, "%x-%x-%x-%x-%x-%x",
                   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
            return false;
        }
    } else {
        char hp[3] = {0};
        for (int i = 0; i < 6; i++) {
            if ((int)strlen(mac_str) < (i * 2 + 2)) {
                return false;
            }
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

static int split_csv_fields(char *line, char **fields, int max_fields) {
    if (!line || !fields || max_fields <= 0) {
        return 0;
    }

    int count = 0;
    fields[count++] = line;
    for (char *p = line; *p && count < max_fields; p++) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

static int find_csv_column(char **fields, int field_count, const char *name) {
    if (!fields || !name) {
        return -1;
    }
    for (int i = 0; i < field_count; i++) {
        if (fields[i] && strcmp(fields[i], name) == 0) {
            return i;
        }
    }
    return -1;
}

void send_alert_notification(const uint8_t target_mac[6], float delta) {
    alert_notify_pkt_t pkt = {0};
    pkt.hdr.type = PKT_ALERT_NOTIFY;
    pkt.hdr.max_hops = 4;
    pkt.hdr.hop_count = 0;
    memcpy(pkt.target_mac, target_mac, 6);
    pkt.delta = delta;
    pkt.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
    ensure_peer(target_mac, current_channel);
    if (reliable_send(target_mac, &pkt, sizeof(pkt), SEND_RETRY_LIMIT)) {
        set_led(255, 0, 255);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_root();
    }
}

static void send_remote_reset_command(const uint8_t target_mac[6]) {
    if (reset_recently_issued(target_mac)) {
        ESP_LOGW(TAG, "Remote reset skipped due to cooldown");
        return;
    }

    if (mac_equal(target_mac, my_mac)) {
        ESP_LOGW(TAG, "Remote reset command targets root/self; executing locally");
        mark_reset_issued(target_mac);
        provisioning_factory_reset();
        return;
    }

    remote_reset_pkt_t pkt = {0};
    pkt.hdr.type = PKT_REMOTE_RESET;
    pkt.hdr.max_hops = 4;
    pkt.hdr.hop_count = 0;
    memcpy(pkt.target_mac, target_mac, 6);
    pkt.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
    pkt.nonce = pkt.timestamp;
    if (pkt.nonce == 0) {
        pkt.nonce = 1;
    }

    ensure_peer(target_mac, current_channel);
    if (reliable_send(target_mac, &pkt, sizeof(pkt), SEND_RETRY_LIMIT)) {
        mark_reset_issued(target_mac);
        ESP_LOGW(TAG,
                 "Remote reset command sent to %02x:%02x:%02x:%02x:%02x:%02x",
                 target_mac[0],
                 target_mac[1],
                 target_mac[2],
                 target_mac[3],
                 target_mac[4],
                 target_mac[5]);
    } else {
        ESP_LOGE(TAG, "Remote reset command send failed");
    }
}

static void process_remote_reset_response(char *response) {
    int sent = 0;
    char *saveptr = NULL;
    char *line = strtok_r(response, "\n", &saveptr);
    int node_idx = -1;
    int value_idx = -1;

    while (line && strstr(line, "node") == NULL) {
        line = strtok_r(NULL, "\n", &saveptr);
    }

    if (!line) {
        return;
    }

    char *header_fields[16] = {0};
    int header_count = split_csv_fields(line, header_fields, 16);
    node_idx = find_csv_column(header_fields, header_count, "node");
    value_idx = find_csv_column(header_fields, header_count, "_value");
    if (node_idx < 0 || value_idx < 0) {
        ESP_LOGW(TAG, "Reset CSV missing node/_value columns");
        return;
    }

    while ((line = strtok_r(NULL, "\n", &saveptr)) != NULL) {
        if (strlen(line) < 5) {
            continue;
        }

        char *fields[12];
        int fc = split_csv_fields(line, fields, 12);

        if (fc > node_idx && fc > value_idx) {
            char *node_mac = fields[node_idx];
            char *value_str = fields[value_idx];
            if (!node_mac || !value_str || *value_str == '\0') {
                continue;
            }

            int code = atoi(value_str);
            if (code != REMOTE_RESET_CODE) {
                continue;
            }

            uint8_t target[6];
            if (!parse_mac_string(node_mac, target)) {
                continue;
            }

            send_remote_reset_command(target);
            sent++;
        }
    }

    if (sent > 0) {
        ESP_LOGW(TAG, "Processed %d remote reset command(s)", sent);
    }
}

static esp_err_t poll_remote_resets_from_influxdb(void) {
    char url[512];
    snprintf(url, sizeof(url), "%s/api/v2/query?org=%s", INFLUXDB_URL, INFLUXDB_ORG);

    char query[768];
    snprintf(query,
             sizeof(query),
             "from(bucket:\"%s\") |> range(start: -%ds) |> filter(fn: (r) => r._measurement == \"command\" and r._field == \"factory_reset_code\" and r._value == %d) |> last() |> keep(columns: [\"_time\", \"node\", \"_value\"]) |> yield()",
             ALERTS_DB,
             REMOTE_RESET_QUERY_WINDOW_S,
             REMOTE_RESET_CODE);

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
        ESP_LOGE(TAG, "HTTP client failed for reset poll");
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
        ESP_LOGE(TAG, "reset open: %s", esp_err_to_name(err));
        esp_http_client_cleanup(c);
        return err;
    }
    int w = esp_http_client_write(c, query, strlen(query));
    if (w <= 0) {
        ESP_LOGE(TAG, "reset write failed");
        esp_http_client_cleanup(c);
        return ESP_FAIL;
    }

    esp_http_client_fetch_headers(c);
    int code = esp_http_client_get_status_code(c);
    char *resp = reset_http_resp_buf;
    int total = 0;
    int rd;
    resp[0] = '\0';
    while ((rd = esp_http_client_read(c, resp + total, ALERT_HTTP_BUF_SIZE - total - 1)) > 0) {
        total += rd;
    }
    resp[total] = '\0';

    if (code == 200 && total > 0) {
        process_remote_reset_response(resp);
    } else if (code != 200) {
        ESP_LOGE(TAG, "Reset HTTP %d", code);
    }

    esp_http_client_cleanup(c);
    return ESP_OK;
}

static void process_remote_share_response(char *response) {
    int toggled = 0;
    char *saveptr = NULL;
    char *line = strtok_r(response, "\n", &saveptr);
    int node_idx = -1;
    int value_idx = -1;

    while (line && strstr(line, "node") == NULL) {
        line = strtok_r(NULL, "\n", &saveptr);
    }

    if (!line) {
        return;
    }

    char *header_fields[16] = {0};
    int header_count = split_csv_fields(line, header_fields, 16);
    node_idx = find_csv_column(header_fields, header_count, "node");
    value_idx = find_csv_column(header_fields, header_count, "_value");
    if (node_idx < 0 || value_idx < 0) {
        return;
    }

    while ((line = strtok_r(NULL, "\n", &saveptr)) != NULL) {
        if (strlen(line) < 5) {
            continue;
        }

        char *fields[12];
        int fc = split_csv_fields(line, fields, 12);
        if (fc <= node_idx || fc <= value_idx) {
            continue;
        }

        char *node_mac = fields[node_idx];
        char *value_str = fields[value_idx];
        if (!node_mac || !value_str || *value_str == '\0') {
            continue;
        }

        int code = atoi(value_str);
        if (code != REMOTE_SHARE_CODE) {
            continue;
        }

        uint8_t target[6];
        if (!parse_mac_string(node_mac, target)) {
            continue;
        }

        if (!mac_equal(target, my_mac)) {
            continue;
        }

        if (share_recently_issued(target)) {
            continue;
        }

        mark_share_issued(target);
        provisioning_toggle_share_mode();
        toggled++;
    }

    if (toggled > 0) {
        ESP_LOGW(TAG, "Processed %d remote share-mode command(s)", toggled);
    }
}

static esp_err_t poll_remote_share_from_influxdb(void) {
    char url[512];
    snprintf(url, sizeof(url), "%s/api/v2/query?org=%s", INFLUXDB_URL, INFLUXDB_ORG);

    char query[768];
    snprintf(query,
             sizeof(query),
             "from(bucket:\"%s\") |> range(start: -%ds) |> filter(fn: (r) => r._measurement == \"command\" and r._field == \"share_mode_code\" and r._value == %d) |> last() |> keep(columns: [\"_time\", \"node\", \"_value\"]) |> yield()",
             ALERTS_DB,
             REMOTE_SHARE_QUERY_WINDOW_S,
             REMOTE_SHARE_CODE);

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
        ESP_LOGE(TAG, "HTTP client failed for share poll");
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
        ESP_LOGE(TAG, "share open: %s", esp_err_to_name(err));
        esp_http_client_cleanup(c);
        return err;
    }
    int w = esp_http_client_write(c, query, strlen(query));
    if (w <= 0) {
        ESP_LOGE(TAG, "share write failed");
        esp_http_client_cleanup(c);
        return ESP_FAIL;
    }

    esp_http_client_fetch_headers(c);
    int code = esp_http_client_get_status_code(c);
    char *resp = share_http_resp_buf;
    int total = 0;
    int rd;
    resp[0] = '\0';
    while ((rd = esp_http_client_read(c, resp + total, ALERT_HTTP_BUF_SIZE - total - 1)) > 0) {
        total += rd;
    }
    resp[total] = '\0';

    if (code == 200 && total > 0) {
        process_remote_share_response(resp);
    } else if (code != 200) {
        ESP_LOGE(TAG, "Share HTTP %d", code);
    }

    esp_http_client_cleanup(c);
    return ESP_OK;
}

static void process_alert_response(char *response) {
    static uint8_t alerted_nodes[5][6] = {{0}};
    static int alerted_count = 0;
    alerted_count = 0;
    int alerts_sent = 0;
    char *saveptr = NULL;
    char *line = strtok_r(response, "\n", &saveptr);
    int node_idx = -1;

    while (line && strstr(line, "node") == NULL) {
        line = strtok_r(NULL, "\n", &saveptr);
    }

    if (!line) {
        ESP_LOGW(TAG, "Alert CSV header not found");
        return;
    }

    char *header_fields[16] = {0};
    int header_count = split_csv_fields(line, header_fields, 16);
    node_idx = find_csv_column(header_fields, header_count, "node");
    if (node_idx < 0) {
        ESP_LOGW(TAG, "Alert CSV missing node column");
        return;
    }

    while ((line = strtok_r(NULL, "\n", &saveptr)) != NULL) {
        if (strlen(line) < 5) {
            continue;
        }
        char *fields[10];
        int fc = split_csv_fields(line, fields, 10);
        if (fc > node_idx) {
            char *node_mac = fields[node_idx];
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
                    if (mac_equal(target, my_mac)) {
                        alert_led_effect();
                        ESP_LOGI(TAG, "Alert for self detected");
                    } else {
                        send_alert_notification(target, 150.0f);
                    }
                    alerts_sent++;
                    if (alerted_count < 5) {
                        memcpy(alerted_nodes[alerted_count], target, 6);
                        alerted_count++;
                    }
                }
            }
        }
    }
    if (alerts_sent > 0) {
        ESP_LOGI(TAG, "Sent %d alert(s)", alerts_sent);
    }
}

static esp_err_t poll_alerts_from_influxdb(void) {
    char url[512];
    snprintf(url, sizeof(url), "%s/api/v2/query?org=%s", INFLUXDB_URL, INFLUXDB_ORG);
    char query[512];
    snprintf(query,
             sizeof(query),
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
    char *resp = alert_http_resp_buf;
    int total = 0, rd;
    resp[0] = '\0';
    while ((rd = esp_http_client_read(c, resp + total, ALERT_HTTP_BUF_SIZE - total - 1)) > 0) {
        total += rd;
    }
    resp[total] = '\0';

    if (code == 200 && total > 0) {
        if (strstr(resp, "true")) {
            process_alert_response(resp);
        }
    } else if (code != 200) {
        ESP_LOGE(TAG, "HTTP %d", code);
    }
    esp_http_client_cleanup(c);

    poll_remote_resets_from_influxdb();
    poll_remote_share_from_influxdb();
    return ESP_OK;
}

#endif 
