#ifndef PROVISIONING_H
#define PROVISIONING_H

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "string.h"
#include "esp_random.h"
#include "mbedtls/gcm.h"
#include "mbedtls/sha256.h"

#if CONFIG_BT_ENABLED
#include "wifi_provisioning/manager.h"
#endif

#define PROV_NAMESPACE                 "spikestop"
#define PROV_KEY_CFG                   "cfg"
#define PROV_KEY_LASTCTR               "lastctr"
#define PROV_KEY_FORCE_ONBOARD         "force_onb"
#define PROV_SHARE_WINDOW_MS           90000
#define PROV_PRODUCT_TAG               "SPIKESTOP"
#define PROV_AES_KEY_LEN               16
#define PROV_IV_LEN                    12
#define PROV_TAG_LEN                   16

#ifndef ALLOW_FALLBACK_WIFI_CREDENTIALS
#define ALLOW_FALLBACK_WIFI_CREDENTIALS 0
#endif

typedef struct {
    uint32_t version;
    uint8_t reserved;
    char wifi_ssid[33];
    char wifi_pass[65];
    char venue_id[40];
    char account_id[40];
} __attribute__((packed)) provisioning_config_t;

extern provisioning_config_t provision_cfg;
extern bool is_provisioned;
extern volatile bool provision_share_mode_active;
extern volatile int64_t provision_share_until_us;
extern uint32_t provision_offer_nonce;
extern uint32_t provision_last_counter;
extern node_role_t role;

static bool provision_force_onboarding = false;

static int64_t now_us(void);
static void set_led(uint8_t r, uint8_t g, uint8_t b);
static void led_root(void);
static void led_child(void);
static void led_isolated(void);

static bool provisioning_is_share_mode_active(void) {
    return provision_share_mode_active;
}

static void provisioning_derive_session_key(uint32_t nonce,
                                            const uint8_t root_mac[6],
                                            const uint8_t child_mac[6],
                                            uint8_t out_key[PROV_AES_KEY_LEN]) {
    uint8_t digest[32] = {0};
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    mbedtls_sha256_update(&ctx, (const unsigned char *)ESPNOW_PMK, strlen(ESPNOW_PMK));
    mbedtls_sha256_update(&ctx, (const unsigned char *)&nonce, sizeof(nonce));
    mbedtls_sha256_update(&ctx, root_mac, 6);
    mbedtls_sha256_update(&ctx, child_mac, 6);
    mbedtls_sha256_finish(&ctx, digest);
    mbedtls_sha256_free(&ctx);
    memcpy(out_key, digest, PROV_AES_KEY_LEN);
}

static esp_err_t provisioning_encrypt_payload(uint32_t nonce,
                                              uint32_t counter,
                                              const uint8_t root_mac[6],
                                              const uint8_t child_mac[6],
                                              const uint8_t *plain,
                                              size_t plain_len,
                                              uint8_t iv[PROV_IV_LEN],
                                              uint8_t tag[PROV_TAG_LEN],
                                              uint8_t *cipher,
                                              size_t cipher_cap,
                                              uint16_t *cipher_len) {
    if (!plain || !cipher || !cipher_len || plain_len > cipher_cap) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t key[PROV_AES_KEY_LEN] = {0};
    provisioning_derive_session_key(nonce, root_mac, child_mac, key);

    esp_fill_random(iv, PROV_IV_LEN);

    uint8_t aad[14] = {0};
    memcpy(aad, &nonce, sizeof(nonce));
    memcpy(aad + sizeof(nonce), &counter, sizeof(counter));
    memcpy(aad + sizeof(nonce) + sizeof(counter), child_mac, 6);

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    int rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, PROV_AES_KEY_LEN * 8);
    if (rc == 0) {
        rc = mbedtls_gcm_crypt_and_tag(&gcm,
                                       MBEDTLS_GCM_ENCRYPT,
                                       plain_len,
                                       iv,
                                       PROV_IV_LEN,
                                       aad,
                                       sizeof(aad),
                                       plain,
                                       cipher,
                                       PROV_TAG_LEN,
                                       tag);
    }
    mbedtls_gcm_free(&gcm);

    if (rc != 0) {
        return ESP_FAIL;
    }

    *cipher_len = (uint16_t)plain_len;
    return ESP_OK;
}

static esp_err_t provisioning_decrypt_payload(uint32_t nonce,
                                              uint32_t counter,
                                              const uint8_t root_mac[6],
                                              const uint8_t child_mac[6],
                                              const uint8_t iv[PROV_IV_LEN],
                                              const uint8_t tag[PROV_TAG_LEN],
                                              const uint8_t *cipher,
                                              size_t cipher_len,
                                              uint8_t *plain,
                                              size_t plain_cap) {
    if (!cipher || !plain || cipher_len > plain_cap) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t key[PROV_AES_KEY_LEN] = {0};
    provisioning_derive_session_key(nonce, root_mac, child_mac, key);

    uint8_t aad[14] = {0};
    memcpy(aad, &nonce, sizeof(nonce));
    memcpy(aad + sizeof(nonce), &counter, sizeof(counter));
    memcpy(aad + sizeof(nonce) + sizeof(counter), child_mac, 6);

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    int rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, PROV_AES_KEY_LEN * 8);
    if (rc == 0) {
        rc = mbedtls_gcm_auth_decrypt(&gcm,
                                      cipher_len,
                                      iv,
                                      PROV_IV_LEN,
                                      aad,
                                      sizeof(aad),
                                      tag,
                                      PROV_TAG_LEN,
                                      cipher,
                                      plain);
    }
    mbedtls_gcm_free(&gcm);

    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t provisioning_load_counter_from_nvs(void) {
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(PROV_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        provision_last_counter = 0;
        return err;
    }

    uint32_t counter = 0;
    err = nvs_get_u32(nvs, PROV_KEY_LASTCTR, &counter);
    nvs_close(nvs);
    if (err == ESP_OK) {
        provision_last_counter = counter;
        return ESP_OK;
    }

    provision_last_counter = 0;
    return err;
}

static esp_err_t provisioning_save_counter_to_nvs(uint32_t counter) {
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(PROV_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u32(nvs, PROV_KEY_LASTCTR, counter);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (err == ESP_OK) {
        provision_last_counter = counter;
    }
    return err;
}

static esp_err_t provisioning_load_from_nvs(void) {
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(PROV_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        is_provisioned = false;
        provision_force_onboarding = false;
        memset(&provision_cfg, 0, sizeof(provision_cfg));
        return err;
    }

    uint8_t force_onboard = 0;
    if (nvs_get_u8(nvs, PROV_KEY_FORCE_ONBOARD, &force_onboard) == ESP_OK) {
        provision_force_onboarding = (force_onboard != 0);
    } else {
        provision_force_onboarding = false;
    }

    size_t required = sizeof(provision_cfg);
    err = nvs_get_blob(nvs, PROV_KEY_CFG, &provision_cfg, &required);
    nvs_close(nvs);

    if (err != ESP_OK || required != sizeof(provision_cfg) || provision_cfg.version == 0) {
        is_provisioned = false;
        memset(&provision_cfg, 0, sizeof(provision_cfg));
        return (err == ESP_OK) ? ESP_ERR_INVALID_SIZE : err;
    }

    provision_cfg.wifi_ssid[sizeof(provision_cfg.wifi_ssid) - 1] = '\0';
    provision_cfg.wifi_pass[sizeof(provision_cfg.wifi_pass) - 1] = '\0';
    provision_cfg.venue_id[sizeof(provision_cfg.venue_id) - 1] = '\0';
    provision_cfg.account_id[sizeof(provision_cfg.account_id) - 1] = '\0';

    is_provisioned = (provision_cfg.wifi_ssid[0] != '\0');
    return is_provisioned ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t provisioning_save_to_nvs(const provisioning_config_t *cfg) {
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(PROV_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(nvs, PROV_KEY_CFG, cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, PROV_KEY_FORCE_ONBOARD, 0);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (err == ESP_OK) {
        provision_cfg = *cfg;
        is_provisioned = (provision_cfg.wifi_ssid[0] != '\0');
        provision_force_onboarding = false;
    }
    return err;
}

static bool provisioning_get_wifi_credentials(char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
    if (!ssid || !pass || ssid_len == 0 || pass_len == 0) {
        return false;
    }

    ssid[0] = '\0';
    pass[0] = '\0';

    if (is_provisioned && provision_cfg.wifi_ssid[0]) {
        strlcpy(ssid, provision_cfg.wifi_ssid, ssid_len);
        strlcpy(pass, provision_cfg.wifi_pass, pass_len);
        return true;
    }

    if (provision_force_onboarding) {
        return false;
    }

#if ALLOW_FALLBACK_WIFI_CREDENTIALS
    if (strlen(ROUTER_SSID) > 0) {
        strlcpy(ssid, ROUTER_SSID, ssid_len);
        strlcpy(pass, ROUTER_PASS, pass_len);
        return true;
    }
#endif

    return false;
}

static void provisioning_start_share_mode(uint32_t duration_ms) {
    provision_share_mode_active = true;
    provision_share_until_us = now_us() + ((int64_t)duration_ms * 1000LL);
    esp_fill_random(&provision_offer_nonce, sizeof(provision_offer_nonce));
    if (provision_offer_nonce == 0) {
        provision_offer_nonce = 1;
    }
    ESP_LOGI(TAG, "Provision share mode ON for %lu ms", (unsigned long)duration_ms);
}

static void provisioning_stop_share_mode(void) {
    provision_share_mode_active = false;
    provision_share_until_us = 0;
    if (role == ROLE_ROOT) {
        led_root();
    } else if (role == ROLE_CHILD) {
        led_child();
    } else {
        led_isolated();
    }
    ESP_LOGI(TAG, "Provision share mode OFF");
}

static void provisioning_toggle_share_mode(void) {
    if (role != ROLE_ROOT) {
        ESP_LOGW(TAG, "Share mode requires ROOT role");
        return;
    }
    if (!is_provisioned) {
        ESP_LOGW(TAG, "Root has no stored provisioning config yet");
        return;
    }

    if (provision_share_mode_active) {
        provisioning_stop_share_mode();
        return;
    }

    provisioning_start_share_mode(PROV_SHARE_WINDOW_MS);
}

static void provisioning_factory_reset(void) {
    ESP_LOGW(TAG, "Factory reset requested: clearing provisioning and Wi-Fi credentials");

    if (provision_share_mode_active) {
        provisioning_stop_share_mode();
    }

    esp_err_t err = nvs_flash_erase();
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        (void)nvs_flash_init();
        err = nvs_flash_erase();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Factory reset: nvs erase failed (%s)", esp_err_to_name(err));
    } else {
        esp_err_t init_err = nvs_flash_init();
        if (init_err != ESP_OK && init_err != ESP_ERR_NVS_NO_FREE_PAGES && init_err != ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW(TAG, "Factory reset: nvs re-init failed (%s)", esp_err_to_name(init_err));
        }
    }

    memset(&provision_cfg, 0, sizeof(provision_cfg));
    is_provisioned = false;
    provision_last_counter = 0;
    provision_force_onboarding = true;

#if CONFIG_BT_ENABLED
    wifi_prov_mgr_reset_provisioning();
#endif

    ESP_LOGW(TAG, "Factory reset complete; rebooting");
    vTaskDelay(pdMS_TO_TICKS(80));
    esp_restart();
}

static void provisioning_housekeeping_task(void *arg) {
    (void)arg;
    while (1) {
        if (provision_share_mode_active && now_us() >= provision_share_until_us) {
            provisioning_stop_share_mode();
        }

        if (provision_share_mode_active) {
            set_led(255, 165, 0);
            vTaskDelay(pdMS_TO_TICKS(120));
            set_led(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(120));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

static void provisioning_start_housekeeping_task(void) {
    static bool started = false;
    if (!started) {
        xTaskCreate(provisioning_housekeeping_task, "prov_housekeep", 3072, NULL, 4, NULL);
        started = true;
    }
}

#endif
