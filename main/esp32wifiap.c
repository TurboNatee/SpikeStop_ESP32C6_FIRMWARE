#include "Package-Imports.h"

static const char *TAG = "MESH_HYBRID";

#include "User-Board-Config.h"

#define ALERT_POLL_INTERVAL_MS 10000
#define DATA_SEND_INTERVAL_MS    5000
#define BUTTON_DEBOUNCE_MS       50
#define BUTTON_LONG_PRESS_MS     2000

typedef enum { ROLE_ISOLATED = 0, ROLE_ROOT = 1, ROLE_CHILD = 2 } node_role_t;

node_role_t role = ROLE_ISOLATED;
uint8_t my_mac[6] = {0};
uint8_t root_mac[6] = {0};
uint8_t parent_mac[6] = {0};
uint8_t current_channel = 1;
uint8_t current_layer = 0;
volatile int64_t last_parent_seen_us = 0;
volatile bool parent_link_up = false;
int8_t best_beacon_rssi = -127;

EventGroupHandle_t app_events;
const int EVT_WIFI_OK = BIT0;
const int EVT_AUTO_SEND = BIT2;
const int EVT_POLL_ALERTS = BIT3;

esp_timer_handle_t data_timer_handle;
esp_timer_handle_t alert_timer_handle;
bool data_timer_running = false;
bool alert_timer_running = false;

#include "LED-Control.h"
#include "Turbidity-Sensor.h"
#include "Utilities.h"
#include "Data-Packets.h"
#include "InfluxDB-Handler.h"
#include "Button-Handler.h"
#include "Alert-Handler.h"
#include "WiFi-Management.h"
#include "ESPNow-Protocol.h"
#include "Mesh-Tasks.h"
#include "Initialization.h"

static void data_timer_callback(void *arg) {
    xEventGroupSetBits(app_events, EVT_AUTO_SEND);
}

static void alert_timer_callback(void *arg) {
    if (role == ROLE_ROOT) {
        xEventGroupSetBits(app_events, EVT_POLL_ALERTS);
    }
}

void init_data_timer(void) {
    esp_timer_create_args_t args = {
        .callback = &data_timer_callback,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "data_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &data_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(data_timer_handle, DATA_SEND_INTERVAL_MS * 1000));
    data_timer_running = true;
}

void init_alert_timer(void) {
    esp_timer_create_args_t args = {
        .callback = &alert_timer_callback,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "alert_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &alert_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(alert_timer_handle, ALERT_POLL_INTERVAL_MS * 1000));
    alert_timer_running = true;
}

static void init_storage(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}

static void init_synchronization(void) {
    pool_mutex = xSemaphoreCreateMutex();
    influxdb_queue = xQueueCreate(30, sizeof(influxdb_data_t *));
    xTaskCreate(influxdb_task, "influxdb_task", 16384, NULL, 4, NULL);
    app_events = xEventGroupCreate();
}

static void init_peripherals(void) {
    init_led_strip();
    init_button();
    turbidity_init();
}

static void update_turbidity_presence(void) {
    float vcheck = turbidity_read_voltage();
    ESP_LOGI(TAG, "Initial turbidity: %.2fV", vcheck);

    bool present = !(vcheck < 1.0f && vcheck > 0.55f);
    turbidity_sensor_present = present;
    if (!present) {
        ESP_LOGW(TAG, "No turbidity sensor detected - continuing");
    }
}

static void init_identity(void) {
    ESP_ERROR_CHECK(esp_read_mac(my_mac, ESP_MAC_WIFI_STA));
    char my[18];
    mac_to_str(my_mac, my, sizeof(my));
    ESP_LOGI(TAG, "MAC %s", my);
}

static void init_espnow(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESPNOW_PMK));
    ESP_ERROR_CHECK(esp_now_register_send_cb((esp_now_send_cb_t)espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb((esp_now_recv_cb_t)espnow_recv_cb));
}

static void start_root_mode(uint8_t ap_ch) {
    role = ROLE_ROOT;
    memcpy(root_mac, my_mac, 6);
    current_channel = ap_ch;
    current_layer = 0;
    parent_link_up = true;
    memcpy(parent_mac, my_mac, 6);
    led_root();
    initialize_sntp();
    xTaskCreate(root_beacon_task, "root_beacon", 4096, NULL, 5, NULL);
    init_alert_timer();
    ESP_LOGI(TAG, "Root ready");
}

static void start_child_mode(void) {
    role = ROLE_ISOLATED;
    current_channel = 1;
    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
    led_isolated();
    xTaskCreate(child_task, "child_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Child scanning");
}

void app_main(void) {
    ESP_LOGI(TAG, "Booting mesh hybrid (clean restart-on-loss)");

    init_peripherals();
    update_turbidity_presence();
    init_synchronization();
    init_storage();
    init_identity();

    bool joined = false;
    uint8_t ap_ch = 1;
    ESP_ERROR_CHECK(wifi_init_sta_or_child(&joined, &ap_ch));
    vTaskDelay(pdMS_TO_TICKS(200));

    init_espnow();

    if (joined) {
        start_root_mode(ap_ch);
    } else {
        start_child_mode();
    }

    init_data_timer();

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            app_events,
            EVT_AUTO_SEND | EVT_POLL_ALERTS,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );
        if (bits & EVT_AUTO_SEND) {
            process_auto_send();
        }
        if ((bits & EVT_POLL_ALERTS) && role == ROLE_ROOT) {
            poll_alerts_from_influxdb();
        }
    }
}
