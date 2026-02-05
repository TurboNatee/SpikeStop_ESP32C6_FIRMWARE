#ifndef WIFI_MANAGEMENT_H
#define WIFI_MANAGEMENT_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

// Forward declarations
void root_beacon_task(void *arg);
void init_data_timer(void);
void init_alert_timer(void);
void initialize_sntp(void);
extern bool data_timer_running;
extern bool alert_timer_running;

static void wifi_reconnect_task(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            ESP_LOGI("MESH_HYBRID", "Child reconnected to router");
            vTaskDelete(NULL);
        } else {
            ESP_LOGI("MESH_HYBRID", "Child attempting router reconnect...");
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_wifi_connect();
        }
    }
}

static void reset_wifi_stack(void) {
    ESP_LOGW(TAG, "Resetting Wi-Fi stack...");

    esp_err_t err;
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_wifi_deinit();
    vTaskDelay(pdMS_TO_TICKS(200));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Wi-Fi reinitialised cleanly");
    } else {
        ESP_LOGE(TAG, "Wi-Fi reinit failed: %s", esp_err_to_name(err));
    }

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(app_events, EVT_WIFI_OK);
        ESP_LOGI(TAG, "Got IP — Wi-Fi connected successfully");

        // Auto-promotion logic
        if (role == ROLE_CHILD || role == ROLE_ISOLATED) {
            ESP_LOGI(TAG, "Child/isolated node connected to router — promoting to ROOT");
            role = ROLE_ROOT;
            memcpy(root_mac, my_mac, 6);
            memcpy(parent_mac, my_mac, 6);
            parent_link_up = true;
            current_layer = 0;
            led_root();

            xTaskCreate(root_beacon_task, "root_beacon", 4096, NULL, 5, NULL);
            if (!data_timer_running) init_data_timer();
            if (!alert_timer_running) init_alert_timer();
            initialize_sntp();

            ESP_LOGI(TAG, "Promoted to ROOT and beaconing...");
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (role == ROLE_ROOT) {
            ESP_LOGW(TAG, "Root lost Wi-Fi — restarting to re-establish router link");
            esp_restart();
        } else {
            ESP_LOGW(TAG, "Child lost Wi-Fi — staying in mesh mode and retrying router every 5s");

            static bool reconnect_task_started = false;
            if (!reconnect_task_started) {
                reconnect_task_started = true;
                xTaskCreate(wifi_reconnect_task, "wifi_reconnect_task", 4096, NULL, 4, NULL);
            }
        }
    }
}

static esp_err_t wifi_init_sta_or_child(bool *joined_router, uint8_t *out_channel) {
    *joined_router = false;
    *out_channel = 1;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t sta = {0};
    strncpy((char*)sta.sta.ssid, ROUTER_SSID, sizeof(sta.sta.ssid));
    strncpy((char*)sta.sta.password, ROUTER_PASS, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    sta.sta.pmf_cfg.capable = false;
    sta.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(app_events, EVT_WIFI_OK, pdTRUE, pdFALSE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & EVT_WIFI_OK) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            *out_channel = ap.primary;
            *joined_router = true;
            ESP_LOGI(TAG, "Connected to router (ch %d)", *out_channel);
        }
    } else {
        ESP_LOGW(TAG, "Router connection timeout — entering child scanning mode");
        reset_wifi_stack();
    }

    return ESP_OK;
}

#endif // WIFI_MANAGEMENT_H
