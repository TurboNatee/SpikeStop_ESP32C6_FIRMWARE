#ifndef INFLUXDB_HANDLER_H
#define INFLUXDB_HANDLER_H

#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "string.h"
#include "time.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// InfluxDB Queue Structure
typedef struct {
    char node_mac[18];
    int sensor_value;
    int temperature;
    int8_t rssi;
    int hops;
    uint64_t timestamp;
} influxdb_data_t;

#define DATA_POOL_SIZE 50
static influxdb_data_t data_pool[DATA_POOL_SIZE];
static int pool_write_idx = 0;
static SemaphoreHandle_t pool_mutex;
static QueueHandle_t influxdb_queue = NULL;

static esp_err_t send_batch_to_influxdb(const char *batch_data) {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi not connected, skip upload");
        return ESP_FAIL;
    }
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGW(TAG, "No netif");
        return ESP_FAIL;
    }
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGW(TAG, "No IP");
        return ESP_FAIL;
    }

    char url[512];
    snprintf(url, sizeof(url), "%s/api/v2/write?org=%s&bucket=%s&precision=s", INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
        .disable_auto_redirect = false,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096,
        .buffer_size_tx = 4096
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        ESP_LOGE(TAG, "HTTP client create failed");
        return ESP_FAIL;
    }

    char auth[300];
    snprintf(auth, sizeof(auth), "Token %s", INFLUXDB_TOKEN);
    esp_http_client_set_header(c, "Authorization", auth);
    esp_http_client_set_header(c, "Content-Type", "text/plain; charset=utf-8");
    esp_http_client_set_header(c, "Accept", "application/json");

    esp_http_client_set_post_field(c, batch_data, strlen(batch_data));
    esp_err_t err = esp_http_client_perform(c);

    if (err == ESP_OK) {
        int code = esp_http_client_get_status_code(c);
        if (code == 204) {
            ESP_LOGI(TAG, "Batch OK");
        } else {
            ESP_LOGE(TAG, "Influx error %d", code);
            char buf[1024] = {0};
            int n = esp_http_client_read(c, buf, sizeof(buf) - 1);
            if (n > 0) ESP_LOGE(TAG, "Resp: %.*s", n, buf);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP perform: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(c);
    return err;
}

static void influxdb_task(void *arg) {
    ESP_LOGI(TAG, "Influx batch task start");
    char batch[4096];
    int batch_count = 0;
    influxdb_data_t *ptr;
    int64_t last_send = time(NULL);

    while (1) {
        if (xQueueReceive(influxdb_queue, &ptr, pdMS_TO_TICKS(5000)) == pdTRUE) {
            batch[0] = '\0';
            batch_count = 0;
            char lp[256];
            char dash[18];
            int j = 0;

            // first point
            for (int i = 0; i < (int)strlen(ptr->node_mac) && j < (int)sizeof(dash) - 1; i++) {
                dash[j++] = (ptr->node_mac[i] == ':') ? '-' : ptr->node_mac[i];
            }
            dash[j] = '\0';

            time_t now = time(NULL);
            if (now < 1000000000) now = 1764614280;
            snprintf(lp, sizeof(lp), "%s,node=%s temperature=%d,sensor_value=%d,rssi=%d,hops=%d %lld\n",
                     INFLUXDB_MEASUREMENT, dash, ptr->temperature, ptr->sensor_value, ptr->rssi, ptr->hops, (long long)now);
            strcat(batch, lp);
            batch_count++;

            for (int k = 0; k < 19; k++) {
                if (xQueueReceive(influxdb_queue, &ptr, 0) == pdTRUE) {
                    j = 0;
                    for (int i = 0; i < (int)strlen(ptr->node_mac) && j < (int)sizeof(dash) - 1; i++) {
                        dash[j++] = (ptr->node_mac[i] == ':') ? '-' : ptr->node_mac[i];
                    }
                    dash[j] = '\0';

                    now = time(NULL);
                    if (now < 1000000000) now = 1764614280;
                    snprintf(lp, sizeof(lp), "%s,node=%s temperature=%d,sensor_value=%d,rssi=%d,hops=%d %lld\n",
                             INFLUXDB_MEASUREMENT, dash, ptr->temperature, ptr->sensor_value, ptr->rssi, ptr->hops, (long long)now);
                    if (strlen(batch) + strlen(lp) < sizeof(batch) - 100) {
                        strcat(batch, lp);
                        batch_count++;
                    } else {
                        xQueueSendToFront(influxdb_queue, &ptr, 0);
                        break;
                    }
                } else {
                    break;
                }
            }

            int64_t nowsec = time(NULL);
            if (batch_count >= 5 || (nowsec - last_send) > 5) {
                for (int r = 0; r < 3; r++) {
                    if (send_batch_to_influxdb(batch) == ESP_OK) {
                        last_send = nowsec;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(800 * (r + 1)));
                }
            } else {
                if (batch_count > 0) {
                    if (send_batch_to_influxdb(batch) == ESP_OK) last_send = nowsec;
                }
            }
        } else {
            if (batch_count > 0) {
                if (send_batch_to_influxdb(batch) == ESP_OK) last_send = time(NULL);
                batch_count = 0;
            }
        }
    }
}

void queue_influxdb_data(const char *node_mac, int sensor_value, int temperature, int8_t rssi, int hops) {
    if (!influxdb_queue || !pool_mutex) {
        ESP_LOGW(TAG, "Queue not ready");
        return;
    }
    xSemaphoreTake(pool_mutex, portMAX_DELAY);
    influxdb_data_t *d = &data_pool[pool_write_idx];
    pool_write_idx = (pool_write_idx + 1) % DATA_POOL_SIZE;
    xSemaphoreGive(pool_mutex);

    d->sensor_value = sensor_value;
    d->temperature = temperature;
    d->rssi = rssi;
    d->hops = hops;
    strncpy(d->node_mac, node_mac, sizeof(d->node_mac) - 1);
    d->node_mac[sizeof(d->node_mac) - 1] = '\0';

    if (xQueueSend(influxdb_queue, &d, pdMS_TO_TICKS(50)) != pdTRUE) {
        influxdb_data_t *drop;
        xQueueReceive(influxdb_queue, &drop, 0);
        xQueueSend(influxdb_queue, &d, pdMS_TO_TICKS(50));
    }
}

#endif // INFLUXDB_HANDLER_H
