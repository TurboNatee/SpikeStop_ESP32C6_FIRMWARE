#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_sntp.h"
#include "secrets.h"

static const char *TAG = "MESH_HYBRID";

// ========= USER CONFIG =========
#define WIFI_CONNECT_TIMEOUT_MS  8000

#define BEACON_INTERVAL_MS       300
#define PARENT_LOSS_MS           15000
#define SEND_RETRY_LIMIT         3

#define LED_PIN                  8   // WS2812 LED
#define BOOT_BUTTON_PIN          9

// ========= InfluxDB Cloud Configuration =========


// ========= Alert Configuration =========
#define ALERTS_INFLUXDB_TOKEN  "fk1D6Ec0SuBr2I3tVPIht1mWQvnfzRJ5_juh85lyWyztgC3bWDnlgFIQmvoDoQJOy2AfJvU4oIpbgauk8cxvrg=="
#define ALERTS_DB              "alerts"
#define ALERT_POLL_INTERVAL_MS 10000  // Check alerts every 10 seconds

// Data sending interval
#define DATA_SEND_INTERVAL_MS    5000  // Send data every 5 seconds

// Button press debounce
#define BUTTON_DEBOUNCE_MS       50
#define BUTTON_LONG_PRESS_MS     2000

// ========= Turbidity Sensor (ADC) =========
// ESP32-C6: ADC_UNIT_1, ADC_CHANNEL_0 = GPIO1
static bool turbidity_sensor_present = false;
#define TURBIDITY_CH ADC_CHANNEL_0   // GPIO1 on ESP32-C6
#define CLEAR_THRESHOLD   2.90f
#define CLOUDY_THRESHOLD  1.50f
static adc_oneshot_unit_handle_t adc_handle;

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

// Alert notification packet
typedef struct {
    mesh_hdr_t hdr;
    uint8_t target_mac[6];
    float delta;
    uint32_t timestamp;
} __attribute__((packed)) alert_notify_pkt_t;

static led_strip_handle_t led_strip;
static QueueHandle_t button_event_queue = NULL;
static volatile int64_t last_button_press_us = 0;
static esp_timer_handle_t data_timer_handle;
static esp_timer_handle_t alert_timer_handle;
static bool data_timer_running = false;
static bool alert_timer_running = false;

// ========= InfluxDB Queue Structure =========
typedef struct {
    char node_mac[18];        // Source node MAC address
    int sensor_value;         // Sensor reading
    int temperature;          // Temperature reading
    int8_t rssi;              // Signal strength
    int hops;                 // Number of hops through mesh
    uint64_t timestamp;       // When data was received
} influxdb_data_t;

// Memory pool for data (no malloc/free)
#define DATA_POOL_SIZE 50
static influxdb_data_t data_pool[DATA_POOL_SIZE];
static int pool_write_idx = 0;
static SemaphoreHandle_t pool_mutex;
static QueueHandle_t influxdb_queue = NULL;

// ========= Node State =========
typedef enum { ROLE_ISOLATED=0, ROLE_ROOT=1, ROLE_CHILD=2 } node_role_t;

static node_role_t role = ROLE_ISOLATED;
static uint8_t my_mac[6] = {0};
static uint8_t root_mac[6] = {0};
static uint8_t parent_mac[6] = {0};
static uint8_t current_channel = 1;
static uint8_t current_layer = 0;
static volatile int64_t last_parent_seen_us = 0;
static volatile bool parent_link_up = false;
static int8_t best_beacon_rssi = -127;

static EventGroupHandle_t app_events;
static const int EVT_WIFI_OK = BIT0;
static const int EVT_AUTO_SEND = BIT2;
static const int EVT_POLL_ALERTS = BIT3;

// Wi-Fi reconnect management
static bool wifi_reconnect_enabled = true;
static int wifi_auth_fail_count = 0;
#define MAX_AUTH_FAILURES 2

// Button event types
typedef enum {
    BUTTON_SHORT_PRESS,
    BUTTON_LONG_PRESS
} button_event_t;

// ========= Function Declarations =========
// Add to function declarations
static void wifi_reconnect_callback(TimerHandle_t timer);
// LED Functions
static void set_led(uint8_t r, uint8_t g, uint8_t b);
static void led_root(void);
static void led_child(void);
static void led_isolated(void);
static void blink_orange(int times);
static void alert_led_effect(void);

// Turbidity Functions
static void turbidity_init(void);
static float turbidity_read_voltage(void);
static int turbidity_get_status(float voltage);

// Timer Functions
static void data_timer_callback(void* arg);
static void alert_timer_callback(void* arg);
static void init_data_timer(void);
static void init_alert_timer(void);

// Helper Functions
static void mac_to_str(const uint8_t m[6], char *out, size_t n);
static bool mac_equal(const uint8_t a[6], const uint8_t b[6]);
static bool mac_is_zero(const uint8_t mac[6]);
static int64_t now_us(void);
static esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel);
static esp_err_t setup_broadcast_peer(uint8_t channel);
static bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries);

// InfluxDB Functions - BATCH UPLOAD
static esp_err_t send_batch_to_influxdb(const char *batch_data);
static void influxdb_task(void *arg);
static void queue_influxdb_data(const char *node_mac, int sensor_value, int temperature, int8_t rssi, int hops);

// Button Functions
static void IRAM_ATTR button_isr_handler(void* arg);
static void button_task(void* arg);

// Data Packet Functions
static void send_data_packet(void);
static void send_sample_data_packet(void);

// Alert Functions
static esp_err_t poll_alerts_from_influxdb(void);
static void process_alert_response(const char* response, int length);
static bool parse_mac_string(const char* mac_str, uint8_t* mac_bytes);
static void send_alert_notification(const uint8_t target_mac[6], float delta);

// WiFi Functions
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data);
static esp_err_t wifi_init_sta_or_child(bool *joined_router, uint8_t *out_channel);
static void wifi_health_task(void *arg);

// ESP-NOW Functions
static void espnow_send_cb(const uint8_t *mac, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);

// Task Functions
static void root_beacon_task(void *arg);
static void child_task(void *arg);

// Initialization Functions
static void init_led_strip(void);
static void init_button(void);
static void initialize_sntp(void);

// Event Processing
static void process_auto_send(void);

// ========= LED Functions =========
static void set_led(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_clear(led_strip);
    vTaskDelay(pdMS_TO_TICKS(5));
    led_strip_set_pixel(led_strip, 0, g, r, b);
    esp_err_t err = led_strip_refresh(led_strip);
    if (err != ESP_OK) {
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(20));
        led_strip_set_pixel(led_strip, 0, g, r, b);
        led_strip_refresh(led_strip);
    }
}

static void led_root() { set_led(0, 0, 255); }
static void led_child() { set_led(0, 255, 0); }
static void led_isolated() { set_led(255, 0, 0); }

static void blink_orange(int times) {
    for (int i = 0; i < times; i++) {
        set_led(255, 165, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
        set_led(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    if (role == ROLE_ROOT) led_root();
    else if (role == ROLE_CHILD) led_child();
    else led_isolated();
}

static void alert_led_effect(void) {
    if (role == ROLE_ROOT) {
        for (int i = 0; i < 10; i++) {
            set_led(255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            set_led(0, 0, 255);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        led_root();
    } else if (role == ROLE_CHILD) {
        for (int i = 0; i < 5; i++) {
            set_led(255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
            set_led(0, 255, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        led_child();
    } else {
        set_led(255, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(3000));
        led_isolated();
    }
}

// ========= Turbidity Sensor Functions =========
static void turbidity_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TURBIDITY_CH, &ch_cfg));
    ESP_LOGI(TAG, "Turbidity sensor initialized on GPIO1 (ADC1_CH0)");
}

static float turbidity_read_voltage(void) {
    int raw = 0;
    adc_oneshot_read(adc_handle, TURBIDITY_CH, &raw);
    float voltage = (raw / 4095.0f) * 3.3f;
    return voltage;
}

static int turbidity_get_status(float voltage) {
    if (voltage > CLEAR_THRESHOLD) return 0;
    else if (voltage > CLOUDY_THRESHOLD) return 1;
    else return 2;
}

// ========= Timer Callbacks =========
static void data_timer_callback(void* arg) {
    xEventGroupSetBits(app_events, EVT_AUTO_SEND);
}

static void alert_timer_callback(void* arg) {
    if (role == ROLE_ROOT) {
        xEventGroupSetBits(app_events, EVT_POLL_ALERTS);
    }
}

static void init_data_timer(void) {
    esp_timer_create_args_t timer_args = {
        .callback = &data_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "data_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &data_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(data_timer_handle, DATA_SEND_INTERVAL_MS * 1000));
    data_timer_running = true;
    ESP_LOGI(TAG, "Data timer initialized - sending every %d ms", DATA_SEND_INTERVAL_MS);
}

static void init_alert_timer(void) {
    esp_timer_create_args_t timer_args = {
        .callback = &alert_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "alert_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &alert_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(alert_timer_handle, ALERT_POLL_INTERVAL_MS * 1000));
    alert_timer_running = true;
    ESP_LOGI(TAG, "Alert timer initialized - polling every %d ms", ALERT_POLL_INTERVAL_MS);
}

// ========= Helpers =========
static void mac_to_str(const uint8_t m[6], char *out, size_t n) {
    snprintf(out, n, "%02x:%02x:%02x:%02x:%02x:%02x",
             m[0],m[1],m[2],m[3],m[4],m[5]);
}

static bool mac_equal(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a,b,6)==0;
}

static bool mac_is_zero(const uint8_t mac[6]) {
    for (int i=0;i<6;i++) if (mac[i]!=0) return false;
    return true;
}

static int64_t now_us(void){ return esp_timer_get_time(); }

static esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel) {
    if (esp_now_is_peer_exist(mac)) return ESP_OK;
    esp_now_peer_info_t p = {0};
    memcpy(p.peer_addr, mac, 6);
    p.channel = channel;
    p.ifidx   = ESP_IF_WIFI_STA;
    p.encrypt = false;
    esp_err_t ret = esp_now_add_peer(&p);
    if (ret != ESP_OK) ESP_LOGE(TAG, "add_peer %s", esp_err_to_name(ret));
    return ret;
}

static esp_err_t setup_broadcast_peer(uint8_t channel) {
    uint8_t broadcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    peer.channel = channel;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    return esp_now_add_peer(&peer);
}

static bool reliable_send(const uint8_t *mac,const void *data,size_t len,int retries){
    for(int i=0;i<retries;i++){
        if (esp_now_send(mac,(uint8_t*)data,len)==ESP_OK){
            vTaskDelay(pdMS_TO_TICKS(50));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100*(i+1)));
    }
    return false;
}

static esp_err_t reset_espnow_for_root_mode(uint8_t channel) {
    ESP_LOGI(TAG, "Performing clean ESP-NOW reset for root mode on channel %d", channel);
    
    // 1. Stop any timers that might interfere
    if (data_timer_running) {
        esp_timer_stop(data_timer_handle);
        data_timer_running = false;
    }
    if (alert_timer_running) {
        esp_timer_stop(alert_timer_handle);
        alert_timer_running = false;
    }
    
    // 2. Reset InfluxDB queue to clear stale data
    if (influxdb_queue != NULL) {
        xQueueReset(influxdb_queue);
        ESP_LOGI(TAG, "Cleared InfluxDB queue of stale data");
    }
    
    // 3. Deinit ESP-NOW completely
    esp_now_deinit();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Crucial wait!
    
    // 4. Ensure Wi-Fi is stable
    ESP_LOGI(TAG, "Waiting for Wi-Fi to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 5. Set Wi-Fi channel
    ESP_LOGI(TAG, "Setting Wi-Fi channel to %d", channel);
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Channel set returned: %s (may be OK if already on channel)", 
                esp_err_to_name(err));
    }
    
    // 6. Wait again for channel to settle
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 7. Re-init ESP-NOW
    ESP_LOGI(TAG, "Re-initializing ESP-NOW...");
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // 8. Reconfigure ESP-NOW
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESPNOW_PMK));
    ESP_ERROR_CHECK(esp_now_register_send_cb((esp_now_send_cb_t)espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb((esp_now_recv_cb_t)espnow_recv_cb));
    
    // 9. Clear ALL existing peers
    uint8_t broadcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_del_peer(broadcast_mac);
    esp_now_del_peer(parent_mac); // Clear old parent if exists
    
    // 10. Setup fresh broadcast peer
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    peer.channel = channel;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "✅ ESP-NOW reset complete for root mode");
    return ESP_OK;
}

// ========= InfluxDB Functions - BATCH UPLOAD =========
static esp_err_t send_batch_to_influxdb(const char *batch_data) {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi not connected, skipping InfluxDB upload");
        return ESP_FAIL;
    }
    
    // Check if we have an IP address
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        ESP_LOGW(TAG, "No network interface, skipping InfluxDB upload");
        return ESP_FAIL;
    }
    
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        ESP_LOGW(TAG, "No IP address, skipping InfluxDB upload");
        return ESP_FAIL;
    }
    
    char url[512];
    snprintf(url, sizeof(url),
             "%s/api/v2/write?org=%s&bucket=%s&precision=s",
             INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
        .disable_auto_redirect = false,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096,
        .buffer_size_tx = 4096,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to create HTTP client");
        return ESP_FAIL;
    }

    char auth_header[300];
    snprintf(auth_header, sizeof(auth_header), "Token %s", INFLUXDB_TOKEN);
    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "text/plain; charset=utf-8");
    esp_http_client_set_header(client, "Accept", "application/json");

    ESP_LOGI(TAG, "Sending batch to InfluxDB (%d bytes)", strlen(batch_data));
    esp_http_client_set_post_field(client, batch_data, strlen(batch_data));

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 204) {
            ESP_LOGI(TAG, "✅ Batch sent successfully");
        } else {
            ESP_LOGE(TAG, "❌ InfluxDB error (Status: %d)", status_code);
            char response_buffer[1024] = {0};
            int content_len = esp_http_client_read(client, response_buffer, sizeof(response_buffer) - 1);
            if (content_len > 0) ESP_LOGE(TAG, "Response: %.*s", content_len, response_buffer);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

static void influxdb_task(void *arg) {
    ESP_LOGI(TAG, "InfluxDB batch task started");
    
    char batch_buffer[4096];
    int batch_count = 0;
    influxdb_data_t *data_ptr;
    int64_t last_send_time = esp_timer_get_time() / 1000000;
    
    while (1) {
        if (xQueueReceive(influxdb_queue, &data_ptr, pdMS_TO_TICKS(5000)) == pdTRUE) {
            batch_buffer[0] = '\0';
            batch_count = 0;
            
            // Process first data point
            char line_protocol[256];
            char node_mac_dash[18];
            
            for (int i = 0, j = 0; i < strlen(data_ptr->node_mac) && j < sizeof(node_mac_dash) - 1; i++) {
                node_mac_dash[j++] = (data_ptr->node_mac[i] == ':') ? '-' : data_ptr->node_mac[i];
                node_mac_dash[j] = '\0';
            }
            
            time_t now = time(NULL);
            if (now < 1000000000) now = 1764614280;
            
            snprintf(line_protocol, sizeof(line_protocol),
                     "%s,node=%s temperature=%d,sensor_value=%d,rssi=%d,hops=%d %lld\n",
                     INFLUXDB_MEASUREMENT,
                     node_mac_dash,
                     data_ptr->temperature,
                     data_ptr->sensor_value,
                     data_ptr->rssi,
                     data_ptr->hops,
                     (long long)now);
            
            strcat(batch_buffer, line_protocol);
            batch_count++;
            
            // Gather more data points (up to 19 more)
            for (int i = 0; i < 19; i++) {
                if (xQueueReceive(influxdb_queue, &data_ptr, 0) == pdTRUE) {
                    for (int i = 0, j = 0; i < strlen(data_ptr->node_mac) && j < sizeof(node_mac_dash) - 1; i++) {
                        node_mac_dash[j++] = (data_ptr->node_mac[i] == ':') ? '-' : data_ptr->node_mac[i];
                        node_mac_dash[j] = '\0';
                    }
                    
                    now = time(NULL);
                    if (now < 1000000000) now = 1764614280;
                    
                    snprintf(line_protocol, sizeof(line_protocol),
                             "%s,node=%s temperature=%d,sensor_value=%d,rssi=%d,hops=%d %lld\n",
                             INFLUXDB_MEASUREMENT,
                             node_mac_dash,
                             data_ptr->temperature,
                             data_ptr->sensor_value,
                             data_ptr->rssi,
                             data_ptr->hops,
                             (long long)now);
                    
                    if (strlen(batch_buffer) + strlen(line_protocol) < sizeof(batch_buffer) - 100) {
                        strcat(batch_buffer, line_protocol);
                        batch_count++;
                    } else {
                        xQueueSendToFront(influxdb_queue, &data_ptr, 0);
                        break;
                    }
                } else {
                    break;
                }
            }
            
            // Send batch if we have 5+ points OR 5 seconds have passed since last send
            int64_t current_time = esp_timer_get_time() / 1000000;
            if (batch_count >= 5 || (current_time - last_send_time) > 5) {
                ESP_LOGI(TAG, "Batch uploading %d data points", batch_count);
                
                for (int retry = 0; retry < 3; retry++) {
                    if (send_batch_to_influxdb(batch_buffer) == ESP_OK) {
                        ESP_LOGI(TAG, "Batch of %d points sent", batch_count);
                        last_send_time = current_time;
                        break;
                    }
                    if (retry < 2) {
                        ESP_LOGW(TAG, "Retrying batch upload (%d/2)...", retry + 1);
                        vTaskDelay(pdMS_TO_TICKS(2000 * (retry + 1)));
                    } else {
                        ESP_LOGE(TAG, "Failed batch after 3 attempts");
                    }
                }
            } else {
                ESP_LOGI(TAG, "Only %d points, waiting for more (min 5)", batch_count);
                // Send small batches anyway to prevent stale data
                if (batch_count > 0) {
                    ESP_LOGI(TAG, "Sending small batch of %d points", batch_count);
                    if (send_batch_to_influxdb(batch_buffer) == ESP_OK) {
                        last_send_time = current_time;
                    }
                }
            }
        } else {
            // Queue empty for 5 seconds, send any partial batch
            if (batch_count > 0) {
                ESP_LOGI(TAG, "Queue empty, sending %d remaining points", batch_count);
                if (send_batch_to_influxdb(batch_buffer) == ESP_OK) {
                    last_send_time = esp_timer_get_time() / 1000000;
                }
                batch_count = 0;
            }
        }
    }
}

static void queue_influxdb_data(const char *node_mac, int sensor_value, 
                               int temperature, int8_t rssi, int hops) {
    if (influxdb_queue == NULL || pool_mutex == NULL) {
        ESP_LOGW(TAG, "Queue not ready");
        return;
    }
    
    xSemaphoreTake(pool_mutex, portMAX_DELAY);
    influxdb_data_t *data = &data_pool[pool_write_idx];
    pool_write_idx = (pool_write_idx + 1) % DATA_POOL_SIZE;
    xSemaphoreGive(pool_mutex);
    
    data->sensor_value = sensor_value;
    data->temperature = temperature;
    data->rssi = rssi;
    data->hops = hops;
    strncpy(data->node_mac, node_mac, sizeof(data->node_mac) - 1);
    data->node_mac[sizeof(data->node_mac) - 1] = '\0';
    
    if (xQueueSend(influxdb_queue, &data, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Queue full - dropping oldest, keeping newest");
        influxdb_data_t *dummy;
        xQueueReceive(influxdb_queue, &dummy, 0);
        
        if (xQueueSend(influxdb_queue, &data, pdMS_TO_TICKS(50)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed even after making space!");
        } else {
            ESP_LOGI(TAG, "Queued (replaced oldest)");
        }
    } else {
        ESP_LOGI(TAG, "Queued data from %s", node_mac);
    }
}

// ========= Button Handling =========
static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t now = now_us();
    if ((now - last_button_press_us) > BUTTON_DEBOUNCE_MS * 1000) {
        last_button_press_us = now;
        button_event_t event = BUTTON_SHORT_PRESS;
        xQueueSendFromISR(button_event_queue, &event, NULL);
    }
}

static void button_task(void* arg) {
    button_event_t event;
    int64_t press_start_time = 0;
    bool button_pressed = false;
    
    while (1) {
        if (xQueueReceive(button_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            if (event == BUTTON_SHORT_PRESS) {
                press_start_time = now_us();
                button_pressed = true;
                
                while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if ((now_us() - press_start_time) > (BUTTON_LONG_PRESS_MS * 1000)) {
                        ESP_LOGI(TAG, "Button long press detected");
                        break;
                    }
                }
                
                if (button_pressed) {
                    button_pressed = false;
                    
                    // Short press only - send SAMPLE data
                    if ((now_us() - press_start_time) < (BUTTON_LONG_PRESS_MS * 1000)) {
                        ESP_LOGI(TAG, "Button short press - sending SAMPLE data packet");
                        send_sample_data_packet();  // Sends sample data, not real data
                    }
                    // Long press could be used for other functions
                }
            }
        }
    }
}

static void send_sample_data_packet(void) {
    // ALWAYS send sample data to database when button is pressed
    char mac_str[18];
    mac_to_str(my_mac, mac_str, sizeof(mac_str));
    
    // SAMPLE DATA 
    float sample_voltage = 2.50f; // Example: 2.50V
    int sample_sensor_value = 250; // 2.50V * 100
    int sample_temp = 22; // Example temperature
    int sample_status = 0; // CLEAR
    
    const char *state_str = (sample_status == 0) ? "CLEAR" : 
                            (sample_status == 1) ? "CLOUDY" : "DIRTY";
    
    ESP_LOGI(TAG, "BUTTON: Sending SAMPLE data to database: %.2fV (%s) -> SENSOR:%d TEMP:%dC",
             sample_voltage, state_str, sample_sensor_value, sample_temp);
    
    // ========= CRITICAL: Direct database upload =========
    queue_influxdb_data(mac_str, sample_sensor_value, sample_temp, -65, 0);
    ESP_LOGI(TAG, "Sample data queued for InfluxDB");
    
    // Also send via mesh if connected (optional)
    if (parent_link_up && !mac_is_zero(parent_mac)) {
        data_pkt_t d = {0};
        d.hdr.type = PKT_DATA;
        d.hdr.max_hops = 8;
        memcpy(d.src_mac, my_mac, 6);
        
        snprintf((char*)d.payload, sizeof(d.payload),
                 "Node:%02x%02x SENSOR:%d TEMP:%dC STATUS:%s SAMPLE",
                 my_mac[4], my_mac[5], sample_sensor_value, sample_temp, state_str);
        
        ensure_peer(parent_mac, current_channel);
        if (reliable_send(parent_mac, &d, sizeof(d), SEND_RETRY_LIMIT)) {
            ESP_LOGI(TAG, "Sample data also sent via mesh");
        }
    }
    
    // Blink YELLOW to indicate sample data
    set_led(255, 255, 0); // Yellow
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Restore role LED
    if (role == ROLE_ROOT) led_root();
    else if (role == ROLE_CHILD) led_child();
    else led_isolated();
}

// ========= Send Data Packet Function =========
static void send_data_packet(void) {
    if (parent_link_up && !mac_is_zero(parent_mac)) {
        data_pkt_t d = {0};
        d.hdr.type = PKT_DATA;
        d.hdr.max_hops = 8;
        memcpy(d.src_mac, my_mac, 6);
        
        float voltage = turbidity_read_voltage();
        
        if (!turbidity_sensor_present) {
            ESP_LOGW(TAG, "Skipping turbidity read — no sensor detected at startup.");
            return;
        }

        int status = turbidity_get_status(voltage);
        int sensor_value = (int)(voltage * 100.0f);
        int temp = 25;
        const char *state_str = (status == 0) ? "CLEAR" : (status == 1) ? "CLOUDY" : "DIRTY";

        snprintf((char*)d.payload, sizeof(d.payload),
                 "Node:%02x%02x SENSOR:%d TEMP:%dC STATUS:%s",
                 my_mac[4], my_mac[5], sensor_value, temp, state_str);

        ESP_LOGI(TAG, "Turbidity: %.2fV (%s) -> SENSOR:%d TEMP:%dC",
                 voltage, state_str, sensor_value, temp);

        ensure_peer(parent_mac, current_channel);
        
        if (reliable_send(parent_mac, &d, sizeof(d), SEND_RETRY_LIMIT)) {
            ESP_LOGI(TAG, "Data packet sent");
            set_led(255, 255, 255);
            vTaskDelay(pdMS_TO_TICKS(100));
            if (role == ROLE_CHILD) led_child();
            else if (role == ROLE_ROOT) led_root();
        }
    } else if (role == ROLE_ISOLATED) {
        ESP_LOGW(TAG, "Cannot send: no parent connection");
        for (int i = 0; i < 3; i++) {
            led_isolated();
            vTaskDelay(pdMS_TO_TICKS(100));
            set_led(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        led_isolated();
    }
}

// ========= Alert Polling Functions =========
static bool parse_mac_string(const char* mac_str, uint8_t* mac_bytes) {
    int values[6];
    
    if (strchr(mac_str, ':')) {
        if (sscanf(mac_str, "%x:%x:%x:%x:%x:%x",
                   &values[0], &values[1], &values[2],
                   &values[3], &values[4], &values[5]) != 6) {
            return false;
        }
    } else if (strchr(mac_str, '-')) {
        if (sscanf(mac_str, "%x-%x-%x-%x-%x-%x",
                   &values[0], &values[1], &values[2],
                   &values[3], &values[4], &values[5]) != 6) {
            return false;
        }
    } else {
        char hex_pair[3] = {0};
        for (int i = 0; i < 6; i++) {
            if (strlen(mac_str) < (i*2 + 2)) return false;
            hex_pair[0] = mac_str[i*2];
            hex_pair[1] = mac_str[i*2 + 1];
            values[i] = strtol(hex_pair, NULL, 16);
        }
    }
    
    for (int i = 0; i < 6; i++) {
        mac_bytes[i] = (uint8_t)values[i];
    }
    return true;
}

static void send_alert_notification(const uint8_t target_mac[6], float delta) {
    ESP_LOGI(TAG, "Sending alert to MAC %02x:%02x:%02x:%02x:%02x:%02x (Δ=%.1f)",
             target_mac[0], target_mac[1], target_mac[2],
             target_mac[3], target_mac[4], target_mac[5], delta);
    
    alert_notify_pkt_t alert_pkt = {0};
    alert_pkt.hdr.type = PKT_ALERT_NOTIFY;
    alert_pkt.hdr.max_hops = 8;
    alert_pkt.hdr.hop_count = 0;
    memcpy(alert_pkt.target_mac, target_mac, 6);
    alert_pkt.delta = delta;
    alert_pkt.timestamp = (uint32_t)(esp_timer_get_time() / 1000000);
    
    ensure_peer(target_mac, current_channel);
    
    if (reliable_send(target_mac, &alert_pkt, sizeof(alert_pkt), SEND_RETRY_LIMIT)) {
        ESP_LOGI(TAG, "Alert sent");
        set_led(255, 0, 255);
        vTaskDelay(pdMS_TO_TICKS(200));
        led_root();
    } else {
        ESP_LOGE(TAG, "Failed to send alert");
    }
}

static void process_alert_response(const char* response, int length) {
    ESP_LOGI(TAG, "Processing alert response");
    
    char buf[4096];
    if (length > (int)(sizeof(buf) - 1)) length = sizeof(buf) - 1;
    memcpy(buf, response, length);
    buf[length] = '\0';
    
    static uint8_t alerted_nodes[5][6] = {0};
    static int alerted_count = 0;
    alerted_count = 0;
    
    int alerts_sent = 0;
    char *saveptr_line = NULL;
    char *line = strtok_r(buf, "\n", &saveptr_line);
    
    while (line && strstr(line, "node") == NULL) {
        line = strtok_r(NULL, "\n", &saveptr_line);
    }
    
    while ((line = strtok_r(NULL, "\n", &saveptr_line)) != NULL) {
        if (strlen(line) < 5) continue;
        
        char *fields[10];
        int field_count = 0;
        char *token = strtok(line, ",");
        
        while (token && field_count < 10) {
            fields[field_count++] = token;
            token = strtok(NULL, ",");
        }
        
        if (field_count >= 5) {
            char *node_mac = fields[4];
            uint8_t target_mac[6];
            
            if (parse_mac_string(node_mac, target_mac)) {
                bool already_alerted = false;
                for (int i = 0; i < alerted_count; i++) {
                    if (memcmp(alerted_nodes[i], target_mac, 6) == 0) {
                        already_alerted = true;
                        break;
                    }
                }
                
                if (!already_alerted) {
                    ESP_LOGI(TAG, "Found active alert for node: %s", node_mac);
                    send_alert_notification(target_mac, 150.0f);
                    alerts_sent++;
                    
                    if (alerted_count < 5) {
                        memcpy(alerted_nodes[alerted_count], target_mac, 6);
                        alerted_count++;
                    }
                }
            }
        }
    }
    
    if (alerts_sent > 0) {
        ESP_LOGI(TAG, "Sent %d alert(s)", alerts_sent);
    } else {
        ESP_LOGI(TAG, "No active alerts");
    }
}

static esp_err_t poll_alerts_from_influxdb(void) {
    ESP_LOGI(TAG, "Polling alerts from InfluxDB...");

    char url[512];
    snprintf(url, sizeof(url), "%s/api/v2/query?org=%s", INFLUXDB_URL, INFLUXDB_ORG);

    char query[512];
    snprintf(query, sizeof(query),
        "from(bucket:\"%s\") "
        "|> range(start: -10s) "
        "|> filter(fn: (r) => r._measurement == \"alert\" and r._field == \"active\" and r._value == true) "
        "|> last() "
        "|> keep(columns: [\"_time\", \"node\", \"_value\"]) "
        "|> yield()",
        ALERTS_DB);

    ESP_LOGI(TAG, "Flux Query: %s", query);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .disable_auto_redirect = false,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to create HTTP client for alerts");
        return ESP_FAIL;
    }

    char auth_header[300];
    snprintf(auth_header, sizeof(auth_header), "Token %s", ALERTS_INFLUXDB_TOKEN);
    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "application/vnd.flux");
    esp_http_client_set_header(client, "Accept", "application/csv");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");

    esp_err_t err = esp_http_client_open(client, strlen(query));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int wlen = esp_http_client_write(client, query, strlen(query));
    if (wlen <= 0) {
        ESP_LOGE(TAG, "Failed to write query body");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP Status Code: %d", status_code);

    char response_buffer[4096] = {0};
    int total_read = 0;
    int read_bytes;

    while ((read_bytes = esp_http_client_read(client,
                                              response_buffer + total_read,
                                              sizeof(response_buffer) - total_read - 1)) > 0) {
        total_read += read_bytes;
    }
    response_buffer[total_read] = '\0';

    ESP_LOGI(TAG, "Response length: %d bytes", total_read);
    if (total_read > 0) {
        ESP_LOGI(TAG, "Response preview (first 200 chars):\n%.*s",
                 total_read < 200 ? total_read : 200, response_buffer);
    }

    if (status_code == 200 && total_read > 0) {
        if (strstr(response_buffer, "true")) {
            ESP_LOGI(TAG, "Found active alert(s)");
            process_alert_response(response_buffer, total_read);
        } else {
            ESP_LOGW(TAG, "No active alerts found");
        }
    } else if (status_code != 200) {
        ESP_LOGE(TAG, "HTTP error: %d", status_code);
    } else {
        ESP_LOGW(TAG, "No response data");
    }

    esp_http_client_cleanup(client);
    return ESP_OK;
}

// ========= Wi-Fi Health Monitoring Task =========
static void wifi_health_task(void *arg) {
    bool was_connected = false;
    int disconnect_count = 0;
    int64_t last_disconnect_time = 0;
    
    ESP_LOGI(TAG, "Wi-Fi health monitor started");
    
    while (1) {
        wifi_ap_record_t ap;
        bool is_connected = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
        
        if (is_connected && !was_connected) {
            // Just reconnected
            ESP_LOGI(TAG, " Wi-Fi reconnected! RSSI: %d", ap.rssi);
            was_connected = true;
            disconnect_count = 0;
            
            if (role == ROLE_ROOT) {
                // Green blink then solid blue
                set_led(0, 255, 0); // Green
                vTaskDelay(pdMS_TO_TICKS(500));
                led_root(); // Blue
            }
        } 
        else if (!is_connected && was_connected) {
            // Just disconnected
            int64_t now = esp_timer_get_time() / 1000;
            if ((now - last_disconnect_time) > 30000) { // 30 seconds
                disconnect_count = 0; // Reset if >30s since last disconnect
            }
            
            disconnect_count++;
            last_disconnect_time = now;
            ESP_LOGW(TAG, "Wi-Fi disconnected (%d times in window)", disconnect_count);
            was_connected = false;
            
            if (role == ROLE_ROOT) {
                // Blink pattern based on disconnect count
                int blinks = 1 + (disconnect_count % 3);
                for (int i = 0; i < blinks; i++) {
                    set_led(255, 0, 0); // Red
                    vTaskDelay(pdMS_TO_TICKS(300));
                    led_root(); // Blue
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
            }
            
            // After 3 disconnections in short period, try aggressive reconnect
            if (disconnect_count >= 3) {
                ESP_LOGI(TAG, "Multiple disconnections, forcing reconnect...");
                esp_wifi_disconnect();
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_wifi_connect();
            }
        }
        else if (!is_connected) {
            // Still disconnected - slow red pulse
            if (role == ROLE_ROOT) {
                static int pulse = 0;
                static bool increasing = true;
                
                if (increasing) {
                    pulse += 5;
                    if (pulse >= 255) increasing = false;
                } else {
                    pulse -= 5;
                    if (pulse <= 0) increasing = true;
                }
                
                set_led(pulse, 0, 0); // Pulsing red
                vTaskDelay(pdMS_TO_TICKS(50));
                
                // Try reconnect every 30 seconds if still disconnected
                static int64_t last_reconnect_attempt = 0;
                int64_t now = esp_timer_get_time() / 1000;
                if ((now - last_reconnect_attempt) > 30000) {
                    ESP_LOGI(TAG, "Attempting Wi-Fi reconnect...");
                    esp_wifi_connect();
                    last_reconnect_attempt = now;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}

// ========= Wi-Fi Setup =========
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    static int reconnect_attempts = 0;  // Track reconnect attempts
    
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (wifi_reconnect_enabled) {
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        // Reset reconnect attempts when we successfully connect
        reconnect_attempts = 0;
        
        ip_event_got_ip_t* e = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(app_events, EVT_WIFI_OK);
        wifi_auth_fail_count = 0;
        wifi_reconnect_enabled = true;
        
        if (role == ROLE_ROOT) {
            led_root(); // Solid blue when connected as root
            
            
            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                uint8_t new_channel = ap.primary;
                if (new_channel != current_channel) {
                    ESP_LOGI(TAG, "Wi-Fi channel changed from %d to %d - updating ESP-NOW channel", 
                             current_channel, new_channel);
                    current_channel = new_channel;
                    
                    // Update Wi-Fi channel (also affects ESP-NOW)
                    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    
                    // Re-setup broadcast peer with new channel
                    uint8_t broadcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
                    if (esp_now_is_peer_exist(broadcast_mac)) {
                        esp_now_del_peer(broadcast_mac);
                    }
                    setup_broadcast_peer(current_channel);
                    
                    // Update existing parent peer if exists
                    if (!mac_is_zero(parent_mac) && esp_now_is_peer_exist(parent_mac)) {
                        esp_now_del_peer(parent_mac);
                        ensure_peer(parent_mac, current_channel);
                    }
                    
                    ESP_LOGI(TAG, "ESP-NOW updated to channel %d", current_channel);
                }
            }
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconnect = (wifi_event_sta_disconnected_t*)data;
        
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason: %d)", disconnect->reason);
        
        // Check if it's an authentication failure
        if (disconnect->reason == WIFI_REASON_AUTH_FAIL || 
            disconnect->reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
            disconnect->reason == WIFI_REASON_AUTH_EXPIRE) {
            
            wifi_auth_fail_count++;
            ESP_LOGI(TAG, "Wi-Fi auth failed (attempt %d/%d)", 
                     wifi_auth_fail_count, MAX_AUTH_FAILURES);
            
            if (wifi_auth_fail_count >= MAX_AUTH_FAILURES) {
                ESP_LOGI(TAG, "Max auth failures. Temporarily disabling Wi-Fi (5 min).");
                
                // Don't permanently disable! Use a timer to re-enable
                wifi_reconnect_enabled = false;
                
                // Create a timer to re-enable Wi-Fi after 5 minutes
                static TimerHandle_t wifi_reconnect_timer = NULL;
                if (wifi_reconnect_timer == NULL) {
                    wifi_reconnect_timer = xTimerCreate("wifi_reconnect", 
                                                        pdMS_TO_TICKS(300000), // 5 minutes
                                                        pdFALSE, 
                                                        (void*)0, 
                                                        wifi_reconnect_callback);
                }
                xTimerStart(wifi_reconnect_timer, 0);
                
                // If we were root, become a child and look for mesh
                if (role == ROLE_ROOT) {
                    role = ROLE_ISOLATED;
                    ESP_LOGI(TAG, "Root lost Wi-Fi, becoming isolated child");
                    led_isolated();
                    
                    // Reset mesh state
                    parent_link_up = false;
                    memset(parent_mac, 0, 6);
                    memset(root_mac, 0, 6);
                    best_beacon_rssi = -127;
                    
                    // Change to default channel for scanning
                    current_channel = 1;
                    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
                    
                    // Stop alert polling timer
                    if (alert_timer_running) {
                        esp_timer_stop(alert_timer_handle);
                        alert_timer_running = false;
                        ESP_LOGI(TAG, "Stopped alert polling (no longer root)");
                    }
                    
                    // Note: root_beacon_task will exit naturally when role changes
                }
                return;
            }
        } else {
            // Other disconnect reason - reset auth counter
            wifi_auth_fail_count = 0;
        }
        
        // If Wi-Fi disconnects and we're root, try to reconnect
        if (role == ROLE_ROOT && wifi_reconnect_enabled) {
            static int backoff_ms = 2000;
            backoff_ms = (backoff_ms < 30000) ? backoff_ms * 2 : 30000;
            
            ESP_LOGW(TAG, "Root Wi-Fi disconnected. Reconnecting in %dms... (attempt %d)", 
                     backoff_ms, reconnect_attempts + 1);
            
            // Visual feedback: red blinks while trying to reconnect
            for (int i = 0; i < 5; i++) {
                set_led(255, 0, 0); // Red
                vTaskDelay(pdMS_TO_TICKS(100));
                led_root(); // Blue
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            
            reconnect_attempts++;
            
            
            if (reconnect_attempts >= 3) {
                ESP_LOGE(TAG, "⚠️ 3 failed reconnect attempts. Restarting Board!");
                esp_restart();
            }
            
            
            esp_wifi_connect();
        } else if (!wifi_reconnect_enabled) {
            // We've given up on Wi-Fi, stay in child mode
            ESP_LOGI(TAG, "Staying in child mode, not reconnecting to Wi-Fi");
            led_isolated();
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
    sta.sta.pmf_cfg.capable = true;
    sta.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_auth_fail_count = 0;
    wifi_reconnect_enabled = true;

    EventBits_t bits = xEventGroupWaitBits(app_events, EVT_WIFI_OK, pdTRUE, pdFALSE,
                                         pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & EVT_WIFI_OK) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
            *out_channel = ap.primary;
        *joined_router = true;
        ESP_LOGI(TAG, "Connected to router (ch %d) RSSI: %d", *out_channel, ap.rssi);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Router connection timeout");
    return ESP_OK;
}

// ========= ESP-NOW Callbacks =========
static void espnow_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    if (!mac) return;
    
    if (status == ESP_NOW_SEND_SUCCESS) {
        if (!mac_equal(mac, (uint8_t[6]){0xFF,0xFF,0xFF,0xFF,0xFF,0xFF})) {
            parent_link_up = true;
            last_parent_seen_us = now_us();
            ESP_LOGD(TAG, "Send to parent: SUCCESS");
        }
    } else {
        if (!mac_equal(mac, (uint8_t[6]){0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}) && 
            !mac_is_zero(parent_mac) && 
            mac_equal(mac, parent_mac)) {
            
            // Track consecutive failures to parent
            static int parent_send_fail_count = 0;
            parent_send_fail_count++;
            
            ESP_LOGW(TAG, "Send to parent: FAIL (%d in a row)", parent_send_fail_count);
            
            // After 3 consecutive failures, RESTART BOARD
            if (parent_send_fail_count >= 3 && parent_link_up) {
                ESP_LOGE(TAG, "Parent lost (send failures). Restarting...");
                esp_restart();
            }
        }
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data || len < sizeof(mesh_hdr_t)) return;

    int8_t rssi = info->rx_ctrl->rssi;
    const uint8_t *from = info->src_addr;
    mesh_hdr_t *hdr = (mesh_hdr_t*)data;
    hdr->rssi = rssi;

    switch(hdr->type) {
        case PKT_BEACON: {
            if (len < sizeof(beacon_pkt_t)) break;
            const beacon_pkt_t *b = (const beacon_pkt_t*)data;
            
            if (role != ROLE_ROOT) {
                // Check if this beacon is from our EXISTING parent
                if (parent_link_up && mac_equal(from, parent_mac)) {
                    // Update timestamp so we know parent is alive
                    last_parent_seen_us = now_us();
                    
                    if (rssi > best_beacon_rssi) best_beacon_rssi = rssi;
                    
                    // DO NOT RE-JOIN! We are already connected.
                    break;
                }

                // (Added hysteresis of 5dB to prevent hopping)
                bool better = (!parent_link_up) || (rssi > (best_beacon_rssi + 5));
                
                if (better) {
                    best_beacon_rssi = rssi;
                    memcpy(root_mac, b->root_mac, 6);
                    memcpy(parent_mac, from, 6);
                    current_channel = b->channel;
                    
                    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
                    vTaskDelay(pdMS_TO_TICKS(100));

                    ensure_peer(parent_mac, current_channel);
                    
                    join_req_pkt_t rq = {0};
                    rq.hdr.type = PKT_JOIN_REQUEST;
                    rq.hdr.max_hops = 5;
                    memcpy(rq.child_mac, my_mac, 6);
                    
                    reliable_send(parent_mac, &rq, sizeof(rq), SEND_RETRY_LIMIT);
                }
            }
            break;
        }

        case PKT_JOIN_REQUEST: {
            if (len < sizeof(join_req_pkt_t)) break;
            if (role == ROLE_ROOT || role == ROLE_CHILD) {
                ensure_peer(from, current_channel);
                
                join_acc_pkt_t ac = {0};
                ac.hdr.type = PKT_JOIN_ACCEPT;
                ac.hdr.max_hops = 5;
                memcpy(ac.parent_mac, my_mac, 6);
                ac.channel = current_channel;
                ac.layer = current_layer + 1;
                
                reliable_send(from, &ac, sizeof(ac), SEND_RETRY_LIMIT);
            }
            break;
        }

                case PKT_JOIN_ACCEPT: {
            if (role != ROLE_ROOT) {
                const join_acc_pkt_t *ac = (const join_acc_pkt_t*)data;
                
                // If we are already connected to this parent, ignore the accept (duplicates)
                if (role == ROLE_CHILD && mac_equal(parent_mac, ac->parent_mac)) {
                    last_parent_seen_us = now_us();
                    break;
                }

                memcpy(parent_mac, ac->parent_mac, 6);
                current_channel = ac->channel;
                current_layer = ac->layer;
                
                ensure_peer(parent_mac, current_channel);
                parent_link_up = true;
                last_parent_seen_us = now_us();
                role = ROLE_CHILD;
                
                char pm[18]; 
                mac_to_str(parent_mac, pm, sizeof(pm));
                ESP_LOGI(TAG, "Joined parent %s ch%d layer%d", pm, current_channel, current_layer);
                led_child();
            }
            break;
        }

        case PKT_DATA: {
            if (len < sizeof(data_pkt_t)) break;
            const data_pkt_t *dp = (const data_pkt_t*)data;
            last_parent_seen_us = now_us();
            
            if (role == ROLE_ROOT) {
                char src[18]; 
                mac_to_str(dp->src_mac, src, sizeof(src));
                ESP_LOGI(TAG, "DATA %s hops=%d: %s", src, dp->hdr.hop_count, (char*)dp->payload);
                
                int sensor_value = 0;
                int temperature = 0;
                if (sscanf((char*)dp->payload, "Node:%*s SENSOR:%d TEMP:%dC", &sensor_value, &temperature) == 2) {
                    queue_influxdb_data(src, sensor_value, temperature, dp->hdr.rssi, dp->hdr.hop_count);
                } else {
                    ESP_LOGW(TAG, "Failed to parse sensor data from payload: %s", dp->payload);
                }
                
                blink_orange(3);
                
            } else if (parent_link_up && dp->hdr.hop_count < dp->hdr.max_hops) {
                data_pkt_t fwd = *dp; 
                fwd.hdr.hop_count++;
                reliable_send(parent_mac, &fwd, sizeof(fwd), SEND_RETRY_LIMIT);
            }
            break;
        }

        case PKT_ALERT_NOTIFY: {
            if (len < sizeof(alert_notify_pkt_t)) break;
            const alert_notify_pkt_t *alert = (const alert_notify_pkt_t*)data;
            
            if (mac_equal(alert->target_mac, my_mac)) {
                ESP_LOGI(TAG, "ALERT NOTIFICATION RECEIVED! Δ=%.1f", alert->delta);
                alert_led_effect();
                
                if (parent_link_up && alert->hdr.hop_count < alert->hdr.max_hops) {
                    alert_notify_pkt_t fwd = *alert;
                    fwd.hdr.hop_count++;
                    reliable_send(parent_mac, &fwd, sizeof(fwd), SEND_RETRY_LIMIT);
                }
            }
            break;
        }

        default: 
            ESP_LOGW(TAG, "Unknown packet type: %d", hdr->type);
            break;
    }
}

// ========= ROOT Beacon Task =========
static void root_beacon_task(void *arg) {
    ESP_LOGI(TAG, "Root beacon task started on channel %d", current_channel);
    
    // Delete broadcast peer if it exists
    uint8_t broadcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (esp_now_is_peer_exist(broadcast_mac)) {
        esp_now_del_peer(broadcast_mac);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Setup fresh broadcast peer
    esp_err_t setup_err = setup_broadcast_peer(current_channel);
    if (setup_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup broadcast peer: %s", esp_err_to_name(setup_err));
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Broadcast peer setup on channel %d", current_channel);
    
    int beacon_count = 0;
    
    while (true) {
        // Only send beacons if we're still root
        if (role != ROLE_ROOT) {
            ESP_LOGI(TAG, "No longer root, stopping beacon task");
            
            // Clean up broadcast peer
            if (esp_now_is_peer_exist(broadcast_mac)) {
                esp_now_del_peer(broadcast_mac);
            }
            
            vTaskDelete(NULL);
            return;
        }
        
        // Get current Wi-Fi channel
        uint8_t pri = 0; 
        wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        esp_wifi_get_channel(&pri, &sec);
        
        // Update channel if it changed
        if (pri != current_channel && pri != 0) {
            ESP_LOGI(TAG, "Channel changed from %d to %d, updating ESP-NOW", 
                     current_channel, pri);
            current_channel = pri;
            
            // Update broadcast peer channel
            if (esp_now_is_peer_exist(broadcast_mac)) {
                esp_now_del_peer(broadcast_mac);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            setup_broadcast_peer(current_channel);
        }
        
        // Create and send beacon
        beacon_pkt_t b = {0};
        b.hdr.type = PKT_BEACON; 
        b.hdr.max_hops = 8;
        memcpy(b.root_mac, my_mac, 6);
        memcpy(b.parent_mac, my_mac, 6);
        b.channel = current_channel;
        b.layer = 0;

        esp_err_t send_result = esp_now_send(broadcast_mac, (uint8_t*)&b, sizeof(b));
        
        if (send_result == ESP_OK) {
            if (beacon_count % 20 == 0) { // Log every 20 beacons (~6 seconds)
                ESP_LOGD(TAG, "Beacon sent on channel %d", current_channel);
            }
        } else {
            ESP_LOGW(TAG, "Failed to send beacon: %s", esp_err_to_name(send_result));
            
            // If send fails, try to re-setup the peer
            if (send_result == ESP_ERR_ESPNOW_CHAN || send_result == ESP_ERR_ESPNOW_NOT_FOUND) {
                ESP_LOGI(TAG, "Re-setting up broadcast peer...");
                if (esp_now_is_peer_exist(broadcast_mac)) {
                    esp_now_del_peer(broadcast_mac);
                }
                setup_broadcast_peer(current_channel);
            }
        }
        
        beacon_count++;
        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
    }
}


static void child_task(void *arg) {
    char parent_str[18] = "none";
    if (!mac_is_zero(parent_mac)) {
        mac_to_str(parent_mac, parent_str, sizeof(parent_str));
    }
    ESP_LOGI(TAG, "=== CHILD START: role=%d, parent=%s, link=%d, channel=%d ===", 
             role, parent_str, parent_link_up, current_channel);
    
    setup_broadcast_peer(current_channel);
    
    int scan_attempts = 0;
    int64_t last_root_attempt_us = 0;
    const int64_t ROOT_ATTEMPT_INTERVAL_US = 15000000; // 15 seconds
    int64_t last_wifi_check_us = 0;
    const int64_t WIFI_CHECK_INTERVAL_US = 30000000; // 30 seconds
    
    wifi_reconnect_enabled = true;
    wifi_auth_fail_count = 0;
    
    while (true) {
        int64_t now = now_us();
        
        // Every 30 seconds, check if we should be root instead of child
        if ((now - last_wifi_check_us) > WIFI_CHECK_INTERVAL_US) {
            if (role == ROLE_CHILD && parent_link_up) {
                ESP_LOGI(TAG, "Periodic Wi-Fi check: Should I be root instead?");
                
                // Try to connect to Wi-Fi to see if it's available
                wifi_reconnect_enabled = true;
                wifi_auth_fail_count = 0;
                esp_wifi_connect();
                
                // Wait a bit to see if we can connect
                EventBits_t bits = xEventGroupWaitBits(app_events, EVT_WIFI_OK, 
                                                      pdFALSE, pdFALSE, 
                                                      pdMS_TO_TICKS(3000));
                
                if (bits & EVT_WIFI_OK) {
                    // We can connect to Wi-Fi! Become root!
                    wifi_ap_record_t ap;
                    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                        uint8_t new_channel = ap.primary;
                        ESP_LOGI(TAG, "WI-FI AVAILABLE! Becoming root on channel %d", new_channel);
                        
                        if (reset_espnow_for_root_mode(new_channel) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to reset ESP-NOW, staying as child");
                            last_wifi_check_us = now;
                            continue;
                        }
                        
                        role = ROLE_ROOT;
                        current_channel = new_channel;
                        memcpy(root_mac, my_mac, 6);
                        current_layer = 0;
                        parent_link_up = true;
                        memcpy(parent_mac, my_mac, 6);
                        
                        // Clear any stale data in the queue
                        xQueueReset(influxdb_queue);
                        
                        for (int i = 0; i < 5; i++) {
                            set_led(0, 0, 255); // Blue
                            vTaskDelay(pdMS_TO_TICKS(200));
                            set_led(0, 0, 0);
                            vTaskDelay(pdMS_TO_TICKS(200));
                        }
                        led_root();
                        
                        xTaskCreate(root_beacon_task, "root_beacon", 4096, NULL, 5, NULL);
                        xTaskCreate(wifi_health_task, "wifi_health", 4096, NULL, 3, NULL);
                        
                        if (!data_timer_running) {
                            init_data_timer();
                        }
                        if (!alert_timer_running) {
                            init_alert_timer();
                        }
                        
                        initialize_sntp();
                        
                        ESP_LOGI(TAG, "SELF-HEALED SUCCESSFULLY! NOW I AM THE ROOT!");
                        
                        // Kill this child task
                        vTaskDelete(NULL);
                        return;
                    }
                }
            }
            last_wifi_check_us = now;
        }
        
        bool kill_parent = false;
        
        // 1. Check timeout
        if (parent_link_up && (now - last_parent_seen_us) > (int64_t)PARENT_LOSS_MS * 1000) {
            ESP_LOGW(TAG, "PARENT TIMEOUT - KILLING CONNECTION");
            kill_parent = true;
        }
        
        // 2. If we have parent MAC but link is down for too long
        if (!parent_link_up && !mac_is_zero(parent_mac)) {
            if ((now - last_parent_seen_us) > (PARENT_LOSS_MS * 2000)) {
                ESP_LOGW(TAG, "STALE PARENT STATE - CLEARING");
                kill_parent = true;
            }
        }
        
        if (kill_parent) {
            ESP_LOGW(TAG, "PARENT DEAD - RESTARTING BOARD");
            esp_restart();
        }
        
        // ========== BECOME ROOT WHEN ISOLATED ==========
        if (role == ROLE_ISOLATED) {
            // TRY TO BECOME ROOT EVERY 15 SECONDS
            if ((now - last_root_attempt_us) > ROOT_ATTEMPT_INTERVAL_US) {
                ESP_LOGI(TAG, "ATTEMPTING TO BECOME ROOT...");
                
                wifi_reconnect_enabled = true;
                wifi_auth_fail_count = 0;
                
                // FORCE WIFI RECONNECT
                esp_wifi_disconnect();
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_wifi_connect();
                
                // WAIT FOR CONNECTION
                EventBits_t bits = xEventGroupWaitBits(app_events, EVT_WIFI_OK, 
                                                      pdFALSE, pdFALSE, 
                                                      pdMS_TO_TICKS(10000));
                
                if (bits & EVT_WIFI_OK) {
                    // We can connect to Wi-Fi! Become root!
                    wifi_ap_record_t ap;
                    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                        uint8_t new_channel = ap.primary;
                        ESP_LOGI(TAG, "WI-FI AVAILABLE! Becoming root on channel %d", new_channel);
                        
                        if (reset_espnow_for_root_mode(new_channel) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to reset ESP-NOW, staying as child");
                            last_root_attempt_us = now;
                            continue;
                        }
                        
                        // ========== Update global state ==========
                        role = ROLE_ROOT;
                        current_channel = new_channel;
                        memcpy(root_mac, my_mac, 6);
                        current_layer = 0;
                        parent_link_up = true;
                        memcpy(parent_mac, my_mac, 6);
                        
                        // Clear any stale data in the queue
                        xQueueReset(influxdb_queue);
                        
                        // ========== Celebrate with LED ==========
                        for (int i = 0; i < 5; i++) {
                            set_led(0, 0, 255); // Blue
                            vTaskDelay(pdMS_TO_TICKS(200));
                            set_led(0, 0, 0);
                            vTaskDelay(pdMS_TO_TICKS(200));
                        }
                        led_root();
                        
                        
                        xTaskCreate(root_beacon_task, "root_beacon", 4096, NULL, 5, NULL);
                        xTaskCreate(wifi_health_task, "wifi_health", 4096, NULL, 3, NULL);
                        
                        
                        if (!data_timer_running) {
                            init_data_timer();
                        }
                        if (!alert_timer_running) {
                            init_alert_timer();
                        }
                        
                        
                        initialize_sntp();
                        
                        ESP_LOGI(TAG, "SELF-HEALED SUCCESSFULLY! NOW I AM THE ROOT!");
                        
                        vTaskDelete(NULL);
                        return;
                    }
                } else {
                    ESP_LOGW(TAG, "Could not connect to Wi-Fi");
                    
                    // BLINK RED ANGRILY
                    for (int i = 0; i < 3; i++) {
                        set_led(255, 0, 0);
                        vTaskDelay(pdMS_TO_TICKS(300));
                        led_isolated();
                        vTaskDelay(pdMS_TO_TICKS(300));
                    }
                }
                
                last_root_attempt_us = now;
            }
        }
        
        
        // ========== SCAN FOR BEACONS IF ISOLATED ==========
        if (role == ROLE_ISOLATED || !parent_link_up) {
            scan_attempts++;
            
            // LOG STATUS
            if (scan_attempts % 10 == 0) {
                ESP_LOGI(TAG, "Scanning: role=%s, parent_link=%d, channel=%d", 
                         role == ROLE_ISOLATED ? "ISOLATED" : "CHILD",
                         parent_link_up, current_channel);
            }
            
            // SWITCH CHANNELS EVERY 3 SECONDS
            if (scan_attempts % 30 == 0) {
                
                uint8_t new_channel = current_channel + 1;
                if (new_channel > 13) new_channel = 1;
                
                ESP_LOGI(TAG, "Switching to channel %d", new_channel);
                current_channel = new_channel;
                esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
                vTaskDelay(pdMS_TO_TICKS(100));
                
                // UPDATE ESP-NOW
                uint8_t broadcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
                if (esp_now_is_peer_exist(broadcast_mac)) {
                    esp_now_del_peer(broadcast_mac);
                }
                setup_broadcast_peer(current_channel);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void wifi_reconnect_callback(TimerHandle_t timer) {
    wifi_reconnect_enabled = true;
    wifi_auth_fail_count = 0;
    ESP_LOGI(TAG, "Wi-Fi reconnection re-enabled after timeout");
}


static void init_led_strip(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    set_led(0, 0, 0);
}


static void init_button(void) {
    button_event_queue = xQueueCreate(10, sizeof(button_event_t));
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_PIN, button_isr_handler, NULL);
    
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
}

// ========= SNTP Time Sync =========
static void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP...");

    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_init();
    #else
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
        sntp_init();
    #endif

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2016 - 1900)) {
        char strftime_buf[64];
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "System time synchronized: %s", strftime_buf);
    } else {
        ESP_LOGW(TAG, "Failed to sync time via NTP, will use fallback timestamps.");
    }
}

// ========= Process Auto Send Event =========
static void process_auto_send(void) {
    if (role == ROLE_ROOT) {
        float voltage = turbidity_read_voltage();

        if (!turbidity_sensor_present) {
            ESP_LOGW(TAG, "Skipping turbidity read — no sensor detected at startup.");
            return;
        }

        int status = turbidity_get_status(voltage);
        int sensor_value = (int)(voltage * 100.0f);
        int temperature = 25;
        const char *state = (status == 0) ? "CLEAR" : (status == 1) ? "CLOUDY" : "DIRTY";

        char mac_str[18];
        mac_to_str(my_mac, mac_str, sizeof(mac_str));

        ESP_LOGI(TAG, "ROOT auto-send: %.2fV (%s) -> SENSOR:%d TEMP:%dC",
                 voltage, state, sensor_value, temperature);

        queue_influxdb_data(mac_str, sensor_value, temperature, -65, 0);

        set_led(255, 255, 255);
        vTaskDelay(pdMS_TO_TICKS(50));
        led_root();
        
    } else if (role == ROLE_CHILD || role == ROLE_ISOLATED) {
        send_data_packet();
    }
}

// ========= MAIN Application =========
void app_main(void) {
    ESP_LOGI(TAG, "Starting ESP-NOW mesh hybrid with Wi-Fi health monitoring...");
    
    // Initialize LED strip
    init_led_strip();
    
    // Initialize button
    init_button();

    // Initialize turbidity sensor
    turbidity_init();

    // Detect turbidity sensor
    float vcheck = turbidity_read_voltage();
    ESP_LOGI(TAG, "Initial turbidity reading: %.2fV", vcheck);
    
    // More flexible detection
    if (vcheck < 1.0f && vcheck > 0.55f) {
        ESP_LOGW(TAG, "No turbidity sensor detected (%.2f V) — continuing without it.", vcheck);
        turbidity_sensor_present = false;
    } else {
        ESP_LOGI(TAG, "Turbidity sensor detected (%.2f V).", vcheck);
        turbidity_sensor_present = true;
    }
    
    // Initialize memory pool and queue for batch upload
    pool_mutex = xSemaphoreCreateMutex();
    influxdb_queue = xQueueCreate(30, sizeof(influxdb_data_t*));

    ESP_LOGI(TAG, "🚀 ALWAYS STARTING INFLUXDB TASK");
    xTaskCreate(influxdb_task, "influxdb_task", 16384, NULL, 4, NULL);
    ESP_LOGI(TAG, "InfluxDB task started (always running)");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Create event group
    app_events = xEventGroupCreate();
    
    // Get device MAC address
    ESP_ERROR_CHECK(esp_read_mac(my_mac, ESP_MAC_WIFI_STA));
    char my[18]; 
    mac_to_str(my_mac, my, sizeof(my));
    ESP_LOGI(TAG, "Device MAC %s", my);

    // Initialize Wi-Fi
    bool joined = false; 
    uint8_t ap_ch = 1;
    ESP_ERROR_CHECK(wifi_init_sta_or_child(&joined, &ap_ch));
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESPNOW_PMK));
    ESP_ERROR_CHECK(esp_now_register_send_cb((esp_now_send_cb_t)espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb((esp_now_recv_cb_t)espnow_recv_cb));

    if (joined) {
        // ROOT node
        role = ROLE_ROOT;
        memcpy(root_mac, my_mac, 6);
        current_channel = ap_ch;
        current_layer = 0;
        
        ESP_LOGI(TAG, "ROLE ROOT ch%d", current_channel);
        led_root();
        
        vTaskDelay(pdMS_TO_TICKS(2000));
        initialize_sntp();
        
        // Start InfluxDB task with batch processing
        xTaskCreate(influxdb_task, "influxdb_task", 16384, NULL, 4, NULL);
        xTaskCreate(root_beacon_task, "root_beacon", 4096, NULL, 5, NULL);
        
        // Start Wi-Fi health monitoring task (ROOT ONLY)
        xTaskCreate(wifi_health_task, "wifi_health", 4096, NULL, 3, NULL);
        
        // Initialize alert polling timer
        init_alert_timer();
        
        ESP_LOGI(TAG, "Root ready. Batch uploads + Wi-Fi health monitoring + alerts.");
        
    } else {
        // CHILD node
        role = ROLE_ISOLATED;
        current_channel = 1;
        esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
        
        ESP_LOGI(TAG, "ROLE CHILD starting scan ch%d", current_channel);
        led_isolated();
        xTaskCreate(child_task, "child_task", 4096, NULL, 5, NULL);
        
        ESP_LOGI(TAG, "Child node ready.");
    }
    
    // Initialize auto-send timer
    init_data_timer();
    
    ESP_LOGI(TAG, "Mesh network initialized successfully!");

    while (true) {
        EventBits_t bits = xEventGroupWaitBits(app_events, 
                                              EVT_AUTO_SEND | EVT_POLL_ALERTS, 
                                              pdTRUE, pdFALSE, portMAX_DELAY);
        
        if (bits & EVT_AUTO_SEND) {
            ESP_LOGI(TAG, "Auto-send timer");
            process_auto_send();
        }
        
        if (bits & EVT_POLL_ALERTS && role == ROLE_ROOT) {
            ESP_LOGI(TAG, "Polling alerts");
            poll_alerts_from_influxdb();
        }
    }
}