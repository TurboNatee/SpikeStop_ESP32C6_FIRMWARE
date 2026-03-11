#ifndef BLE_PROVISIONING_H
#define BLE_PROVISIONING_H

#include "esp_log.h"

#if CONFIG_BT_ENABLED
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"

extern uint8_t my_mac[6];

static bool ble_provisioning_active = false;

static void ble_prov_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg;
    if (event_base != WIFI_PROV_EVENT) {
        return;
    }

    switch (event_id) {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "BLE provisioning started");
            break;
        case WIFI_PROV_CRED_RECV: {
            wifi_sta_config_t *wifi_cfg = (wifi_sta_config_t *)event_data;
            if (!wifi_cfg) {
                break;
            }
            provisioning_config_t cfg = {0};
            cfg.version = 1;
            strlcpy(cfg.wifi_ssid, (const char *)wifi_cfg->ssid, sizeof(cfg.wifi_ssid));
            strlcpy(cfg.wifi_pass, (const char *)wifi_cfg->password, sizeof(cfg.wifi_pass));
            strlcpy(cfg.venue_id, "unassigned", sizeof(cfg.venue_id));
            strlcpy(cfg.account_id, "unassigned", sizeof(cfg.account_id));
            if (provisioning_save_to_nvs(&cfg) == ESP_OK) {
                ESP_LOGI(TAG, "Provisioning credentials saved from BLE");
            } else {
                ESP_LOGE(TAG, "Failed to save BLE credentials to NVS");
            }
            break;
        }
        case WIFI_PROV_CRED_FAIL:
            ESP_LOGW(TAG, "BLE provisioning credential validation failed");
            break;
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "BLE provisioning credentials accepted");
            break;
        case WIFI_PROV_END:
            wifi_prov_mgr_deinit();
            ble_provisioning_active = false;
            ESP_LOGI(TAG, "BLE provisioning finished; restarting to apply config");
            esp_restart();
            break;
        default:
            break;
    }
}

static void ble_provisioning_start_if_needed(void) {
    if (is_provisioned || ble_provisioning_active) {
        return;
    }

    wifi_prov_mgr_config_t cfg = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };

    ESP_ERROR_CHECK(wifi_prov_mgr_init(cfg));

    bool already = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&already));
    if (already) {
        ESP_LOGW(TAG, "Wi-Fi provisioning manager reports provisioned state; resetting to allow BLE onboarding");
        ESP_ERROR_CHECK(wifi_prov_mgr_reset_provisioning());
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &ble_prov_event_handler, NULL));

    char service_name[24] = {0};
    snprintf(service_name, sizeof(service_name), "SpikeStop_%02X%02X", my_mac[4], my_mac[5]);
    const char *pop = "spikestop-setup";

    ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1,
                                                     (const void *)pop,
                                                     service_name,
                                                     NULL));
    ble_provisioning_active = true;
    ESP_LOGI(TAG, "BLE onboarding active as '%s'", service_name);
}

#else

static void ble_provisioning_start_if_needed(void) {
    if (!is_provisioned) {
        ESP_LOGW(TAG, "BT disabled in sdkconfig; BLE onboarding unavailable");
    }
}

#endif

#endif
