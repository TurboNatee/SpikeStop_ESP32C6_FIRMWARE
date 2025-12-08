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
#include "secrets.h"  // keep your PMK/Influx secrets etc.

static const char *TAG = "MESH_HYBRID";

// ========= USER CONFIG =========
#define WIFI_CONNECT_TIMEOUT_MS  8000
#define BEACON_INTERVAL_MS       300
#define PARENT_LOSS_MS           15000
#define SEND_RETRY_LIMIT         3
#define LED_PIN                  8
#define BOOT_BUTTON_PIN          9

// ========= Alert Configuration =========
#define ALERTS_INFLUXDB_TOKEN  "fk1D6Ec0SuBr2I3tVPIht1mWQvnfzRJ5_juh85lyWyztgC3bWDnlgFIQmvoDoQJOy2AfJvU4oIpbgauk8cxvrg=="
#define ALERTS_DB              "alerts"
#define ALERT_POLL_INTERVAL_MS 10000

// Data sending interval
#define DATA_SEND_INTERVAL_MS    5000

// Button press debounce / long press window
#define BUTTON_DEBOUNCE_MS       50
#define BUTTON_LONG_PRESS_MS     2000

// ========= Turbidity Sensor (ADC) =========
// ESP32-C6: ADC_UNIT_1, ADC_CHANNEL_0 = GPIO1
static bool turbidity_sensor_present = false;
#define TURBIDITY_CH ADC_CHANNEL_0
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

// Button event types
typedef enum { BUTTON_SHORT_PRESS, BUTTON_LONG_PRESS } button_event_t;

// ========= Unified Restart Helper =========
static void restart_node(const char *reason) {
    ESP_LOGE(TAG, "Restarting: %s", reason ? reason : "no-reason");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

// ========= Declarations =========
static void set_led(uint8_t r, uint8_t g, uint8_t b);
static void led_root(void);
static void led_child(void);
static void led_isolated(void);
static void blink_orange(int times);
static void alert_led_effect(void);

static void turbidity_init(void);
static float turbidity_read_voltage(void);
static int turbidity_get_status(float voltage);

static void data_timer_callback(void* arg);
static void alert_timer_callback(void* arg);
static void init_data_timer(void);
static void init_alert_timer(void);

static void mac_to_str(const uint8_t m[6], char *out, size_t n);
static bool mac_equal(const uint8_t a[6], const uint8_t b[6]);
static bool mac_is_zero(const uint8_t mac[6]);
static int64_t now_us(void);
static esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel);
static esp_err_t setup_broadcast_peer(uint8_t channel);
static bool reliable_send(const uint8_t *mac, const void *data, size_t len, int retries);

static esp_err_t send_batch_to_influxdb(const char *batch_data);
static void influxdb_task(void *arg);
static void queue_influxdb_data(const char *node_mac, int sensor_value, int temperature, int8_t rssi, int hops);

static void IRAM_ATTR button_isr_handler(void* arg);
static void button_task(void* arg);

static void send_data_packet(void);
static void send_sample_data_packet(void);

static esp_err_t poll_alerts_from_influxdb(void);
static void process_alert_response(const char* response, int length);
static bool parse_mac_string(const char* mac_str, uint8_t* mac_bytes);
static void send_alert_notification(const uint8_t target_mac[6], float delta);

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data);
static esp_err_t wifi_init_sta_or_child(bool *joined_router, uint8_t *out_channel);

static void espnow_send_cb(const uint8_t *mac, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);

static void root_beacon_task(void *arg);
static void child_task(void *arg);

static void init_led_strip(void);
static void init_button(void);
static void initialize_sntp(void);
static void process_auto_send(void);

// ========= LED =========
static void set_led(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_clear(led_strip);
    vTaskDelay(pdMS_TO_TICKS(5));
    led_strip_set_pixel(led_strip, 0, g, r, b);
    if (led_strip_refresh(led_strip) != ESP_OK) {
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(20));
        led_strip_set_pixel(led_strip, 0, g, r, b);
        led_strip_refresh(led_strip);
    }
}
static void led_root()    { set_led(0, 0, 255); }
static void led_child()   { set_led(0, 255, 0); }
static void led_isolated(){ set_led(255, 0, 0); }

static void blink_orange(int times) {
    for (int i = 0; i < times; i++) {
        set_led(255, 165, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        set_led(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    if (role == ROLE_ROOT) led_root();
    else if (role == ROLE_CHILD) led_child();
    else led_isolated();
}

static void alert_led_effect(void) {
    if (role == ROLE_ROOT) {
        for (int i = 0; i < 10; i++) { set_led(255, 0, 0); vTaskDelay(pdMS_TO_TICKS(100)); set_led(0, 0, 255); vTaskDelay(pdMS_TO_TICKS(100)); }
        led_root();
    } else if (role == ROLE_CHILD) {
        for (int i = 0; i < 5; i++) { set_led(255, 0, 0); vTaskDelay(pdMS_TO_TICKS(300)); set_led(0, 255, 0); vTaskDelay(pdMS_TO_TICKS(300)); }
        led_child();
    } else { set_led(255, 0, 0); vTaskDelay(pdMS_TO_TICKS(1500)); led_isolated(); }
}

// ========= Turbidity =========
static void turbidity_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t ch_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TURBIDITY_CH, &ch_cfg));
    ESP_LOGI(TAG, "Turbidity ADC init @ GPIO1");
}
static float turbidity_read_voltage(void) {
    int raw = 0; adc_oneshot_read(adc_handle, TURBIDITY_CH, &raw);
    return (raw / 4095.0f) * 3.3f;
}
static int turbidity_get_status(float v) {
    if (v > CLEAR_THRESHOLD) return 0;
    else if (v > CLOUDY_THRESHOLD) return 1;
    else return 2;
}

// ========= Timers =========
static void data_timer_callback(void* arg)  { xEventGroupSetBits(app_events, EVT_AUTO_SEND); }
static void alert_timer_callback(void* arg) { if (role == ROLE_ROOT) xEventGroupSetBits(app_events, EVT_POLL_ALERTS); }

static void init_data_timer(void) {
    esp_timer_create_args_t args = {.callback=&data_timer_callback,.dispatch_method=ESP_TIMER_TASK,.name="data_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&args, &data_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(data_timer_handle, DATA_SEND_INTERVAL_MS * 1000));
    data_timer_running = true;
}
static void init_alert_timer(void) {
    esp_timer_create_args_t args = {.callback=&alert_timer_callback,.dispatch_method=ESP_TIMER_TASK,.name="alert_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&args, &alert_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(alert_timer_handle, ALERT_POLL_INTERVAL_MS * 1000));
    alert_timer_running = true;
}

// ========= Helpers =========
static void mac_to_str(const uint8_t m[6], char *out, size_t n) { snprintf(out, n, "%02x:%02x:%02x:%02x:%02x:%02x", m[0],m[1],m[2],m[3],m[4],m[5]); }
static bool mac_equal(const uint8_t a[6], const uint8_t b[6]) { return memcmp(a,b,6)==0; }
static bool mac_is_zero(const uint8_t mac[6]) { for (int i=0;i<6;i++) if (mac[i]) return false; return true; }
static int64_t now_us(void){ return esp_timer_get_time(); }

static esp_err_t ensure_peer(const uint8_t mac[6], uint8_t channel) {
    if (esp_now_is_peer_exist(mac)) return ESP_OK;
    esp_now_peer_info_t p = {0}; memcpy(p.peer_addr, mac, 6);
    p.channel = channel; p.ifidx = ESP_IF_WIFI_STA; p.encrypt = false;
    esp_err_t r = esp_now_add_peer(&p);
    if (r != ESP_OK) ESP_LOGE(TAG,"add_peer %s", esp_err_to_name(r));
    return r;
}
static esp_err_t setup_broadcast_peer(uint8_t channel) {
    uint8_t b[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (esp_now_is_peer_exist(b)) esp_now_del_peer(b);
    esp_now_peer_info_t p = {0}; memcpy(p.peer_addr, b, 6);
    p.channel = channel; p.ifidx = ESP_IF_WIFI_STA; p.encrypt = false;
    return esp_now_add_peer(&p);
}
static bool reliable_send(const uint8_t *mac,const void *data,size_t len,int retries){
    for(int i=0;i<retries;i++){ if (esp_now_send(mac,(uint8_t*)data,len)==ESP_OK){ vTaskDelay(pdMS_TO_TICKS(20)); return true; } vTaskDelay(pdMS_TO_TICKS(60*(i+1))); }
    return false;
}

// ========= InfluxDB (unchanged behaviour) =========
static esp_err_t send_batch_to_influxdb(const char *batch_data) {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) { ESP_LOGW(TAG, "Wi-Fi not connected, skip upload"); return ESP_FAIL; }
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) { ESP_LOGW(TAG, "No netif"); return ESP_FAIL; }
    esp_netif_ip_info_t ip; if (esp_netif_get_ip_info(netif, &ip) != ESP_OK) { ESP_LOGW(TAG,"No IP"); return ESP_FAIL; }

    char url[512];
    snprintf(url, sizeof(url), "%s/api/v2/write?org=%s&bucket=%s&precision=s", INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET);

    esp_http_client_config_t cfg = {.url=url,.method=HTTP_METHOD_POST,.timeout_ms=15000,.disable_auto_redirect=false,.crt_bundle_attach=esp_crt_bundle_attach,.buffer_size=4096,.buffer_size_tx=4096};
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) { ESP_LOGE(TAG,"HTTP client create failed"); return ESP_FAIL; }

    char auth[300]; snprintf(auth,sizeof(auth),"Token %s", INFLUXDB_TOKEN);
    esp_http_client_set_header(c,"Authorization",auth);
    esp_http_client_set_header(c,"Content-Type","text/plain; charset=utf-8");
    esp_http_client_set_header(c,"Accept","application/json");

    esp_http_client_set_post_field(c, batch_data, strlen(batch_data));
    esp_err_t err = esp_http_client_perform(c);

    if (err == ESP_OK) {
        int code = esp_http_client_get_status_code(c);
        if (code == 204) { ESP_LOGI(TAG,"Batch OK"); }
        else { ESP_LOGE(TAG,"Influx error %d", code); char buf[1024]={0}; int n=esp_http_client_read(c,buf,sizeof(buf)-1); if(n>0) ESP_LOGE(TAG,"Resp: %.*s",n,buf); err=ESP_FAIL; }
    } else {
        ESP_LOGE(TAG,"HTTP perform: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(c);
    return err;
}

static void influxdb_task(void *arg) {
    ESP_LOGI(TAG,"Influx batch task start");
    char batch[4096]; int batch_count=0;
    influxdb_data_t *ptr; int64_t last_send=time(NULL);

    while (1) {
        if (xQueueReceive(influxdb_queue, &ptr, pdMS_TO_TICKS(5000)) == pdTRUE) {
            batch[0]='\0'; batch_count=0; char lp[256]; char dash[18];
            // first point
            for (int i=0,j=0;i<(int)strlen(ptr->node_mac)&&j<(int)sizeof(dash)-1;i++){ dash[j++]=(ptr->node_mac[i]==':')?'-':ptr->node_mac[i]; dash[j]='\0'; }
            time_t now=time(NULL); if (now<1000000000) now=1764614280;
            snprintf(lp,sizeof(lp), "%s,node=%s temperature=%d,sensor_value=%d,rssi=%d,hops=%d %lld\n",
                     INFLUXDB_MEASUREMENT,dash,ptr->temperature,ptr->sensor_value,ptr->rssi,ptr->hops,(long long)now);
            strcat(batch,lp); batch_count++;

            for (int k=0;k<19;k++){
                if (xQueueReceive(influxdb_queue,&ptr,0)==pdTRUE){
                    for (int i=0,j=0;i<(int)strlen(ptr->node_mac)&&j<(int)sizeof(dash)-1;i++){ dash[j++]=(ptr->node_mac[i]==':')?'-':ptr->node_mac[i]; dash[j]='\0'; }
                    now=time(NULL); if (now<1000000000) now=1764614280;
                    snprintf(lp,sizeof(lp), "%s,node=%s temperature=%d,sensor_value=%d,rssi=%d,hops=%d %lld\n",
                             INFLUXDB_MEASUREMENT,dash,ptr->temperature,ptr->sensor_value,ptr->rssi,ptr->hops,(long long)now);
                    if (strlen(batch)+strlen(lp) < sizeof(batch)-100){ strcat(batch,lp); batch_count++; }
                    else { xQueueSendToFront(influxdb_queue,&ptr,0); break; }
                } else break;
            }

            int64_t nowsec = time(NULL);
            if (batch_count>=5 || (nowsec - last_send) > 5){
                for (int r=0;r<3;r++){ if (send_batch_to_influxdb(batch)==ESP_OK){ last_send=nowsec; break; } vTaskDelay(pdMS_TO_TICKS(800*(r+1))); }
            } else {
                if (batch_count>0){ if (send_batch_to_influxdb(batch)==ESP_OK) last_send=nowsec; }
            }
        } else {
            if (batch_count>0){ if (send_batch_to_influxdb(batch)==ESP_OK) last_send=time(NULL); batch_count=0; }
        }
    }
}

static void queue_influxdb_data(const char *node_mac, int sensor_value,int temperature,int8_t rssi,int hops) {
    if (!influxdb_queue || !pool_mutex) { ESP_LOGW(TAG,"Queue not ready"); return; }
    xSemaphoreTake(pool_mutex, portMAX_DELAY);
    influxdb_data_t *d = &data_pool[pool_write_idx];
    pool_write_idx = (pool_write_idx + 1) % DATA_POOL_SIZE;
    xSemaphoreGive(pool_mutex);

    d->sensor_value=sensor_value; d->temperature=temperature; d->rssi=rssi; d->hops=hops;
    strncpy(d->node_mac, node_mac, sizeof(d->node_mac)-1); d->node_mac[sizeof(d->node_mac)-1]='\0';

    if (xQueueSend(influxdb_queue, &d, pdMS_TO_TICKS(50)) != pdTRUE) {
        influxdb_data_t *drop; xQueueReceive(influxdb_queue,&drop,0);
        xQueueSend(influxdb_queue, &d, pdMS_TO_TICKS(50));
    }
}

// ========= Button =========
static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t n = now_us();
    if ((n - last_button_press_us) > BUTTON_DEBOUNCE_MS*1000) {
        last_button_press_us = n;
        button_event_t e = BUTTON_SHORT_PRESS;
        xQueueSendFromISR(button_event_queue, &e, NULL);
    }
}
static void button_task(void* arg) {
    button_event_t e; int64_t start=0; bool pressed=false;
    while (1) {
        if (xQueueReceive(button_event_queue,&e,portMAX_DELAY)==pdTRUE) {
            if (e==BUTTON_SHORT_PRESS) {
                start=now_us(); pressed=true;
                while (gpio_get_level(BOOT_BUTTON_PIN)==0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if ((now_us()-start) > BUTTON_LONG_PRESS_MS*1000) break;
                }
                if (pressed) {
                    pressed=false;
                    if ((now_us()-start) < BUTTON_LONG_PRESS_MS*1000) {
                        send_sample_data_packet();
                    }
                }
            }
        }
    }
}

static void send_sample_data_packet(void) {
    char mac_str[18]; mac_to_str(my_mac, mac_str, sizeof(mac_str));
    float sample_voltage=2.50f; int sample_sensor_value=250; int sample_temp=22;
    queue_influxdb_data(mac_str, sample_sensor_value, sample_temp, -65, 0);

    if (parent_link_up && !mac_is_zero(parent_mac)) {
        data_pkt_t d={0}; d.hdr.type=PKT_DATA; d.hdr.max_hops=8; memcpy(d.src_mac,my_mac,6);
        snprintf((char*)d.payload,sizeof(d.payload),"Node:%02x%02x SENSOR:%d TEMP:%dC STATUS:%s SAMPLE",
                 my_mac[4], my_mac[5], sample_sensor_value, sample_temp, "CLEAR");
        ensure_peer(parent_mac, current_channel);
        reliable_send(parent_mac, &d, sizeof(d), SEND_RETRY_LIMIT);
    }
    set_led(255,255,0); vTaskDelay(pdMS_TO_TICKS(100));
    if (role==ROLE_ROOT) led_root(); else if (role==ROLE_CHILD) led_child(); else led_isolated();
}

// ========= Data Send =========
static void send_data_packet(void) {
    if (parent_link_up && !mac_is_zero(parent_mac)) {
        float v = turbidity_read_voltage();
        if (!turbidity_sensor_present) { ESP_LOGW(TAG,"No turbidity sensor — skip"); return; }
        int status = turbidity_get_status(v);
        int sensor_value=(int)(v*100.0f); int temp=25;
        const char *state_str = (status==0)?"CLEAR":(status==1)?"CLOUDY":"DIRTY";

        data_pkt_t d={0}; d.hdr.type=PKT_DATA; d.hdr.max_hops=8; memcpy(d.src_mac,my_mac,6);
        snprintf((char*)d.payload,sizeof(d.payload),"Node:%02x%02x SENSOR:%d TEMP:%dC STATUS:%s",
                 my_mac[4], my_mac[5], sensor_value, temp, state_str);

        ensure_peer(parent_mac, current_channel);
        if (reliable_send(parent_mac, &d, sizeof(d), SEND_RETRY_LIMIT)) {
            set_led(255,255,255); vTaskDelay(pdMS_TO_TICKS(60));
            if (role==ROLE_CHILD) led_child(); else if (role==ROLE_ROOT) led_root();
        }
    } else if (role==ROLE_ISOLATED) {
        for (int i=0;i<2;i++){ led_isolated(); vTaskDelay(pdMS_TO_TICKS(100)); set_led(0,0,0); vTaskDelay(pdMS_TO_TICKS(100)); }
        led_isolated();
    }
}

// ========= Alerts (unchanged behaviour) =========
static bool parse_mac_string(const char* mac_str, uint8_t* mac_bytes) {
    int v[6];
    if (strchr(mac_str, ':')) {
        if (sscanf(mac_str,"%x:%x:%x:%x:%x:%x",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5])!=6) return false;
    } else if (strchr(mac_str,'-')) {
        if (sscanf(mac_str,"%x-%x-%x-%x-%x-%x",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5])!=6) return false;
    } else {
        char hp[3]={0}; for(int i=0;i<6;i++){ if ((int)strlen(mac_str)<(i*2+2)) return false; hp[0]=mac_str[i*2]; hp[1]=mac_str[i*2+1]; v[i]=strtol(hp,NULL,16); }
    }
    for (int i = 0; i < 6; i++) {
    mac_bytes[i] = (uint8_t)v[i];
}
return true;

}

static void send_alert_notification(const uint8_t target_mac[6], float delta) {
    alert_notify_pkt_t pkt={0}; pkt.hdr.type=PKT_ALERT_NOTIFY; pkt.hdr.max_hops=8; pkt.hdr.hop_count=0;
    memcpy(pkt.target_mac,target_mac,6); pkt.delta=delta; pkt.timestamp = (uint32_t)(esp_timer_get_time()/1000000);
    ensure_peer(target_mac, current_channel);
    if (reliable_send(target_mac,&pkt,sizeof(pkt),SEND_RETRY_LIMIT)) { set_led(255,0,255); vTaskDelay(pdMS_TO_TICKS(150)); led_root(); }
}

static void process_alert_response(const char* response, int length) {
    char buf[4096]; if (length>(int)sizeof(buf)-1) length=sizeof(buf)-1; memcpy(buf,response,length); buf[length]='\0';
    static uint8_t alerted_nodes[5][6]={{0}}; static int alerted_count=0; alerted_count=0;
    int alerts_sent=0; char *saveptr=NULL; char *line=strtok_r(buf,"\n",&saveptr);
    while (line && strstr(line,"node")==NULL) line=strtok_r(NULL,"\n",&saveptr);
    while ((line=strtok_r(NULL,"\n",&saveptr))!=NULL){
        if (strlen(line)<5) continue;
        char *fields[10]; int fc=0; char *t=strtok(line,",");
        while (t && fc<10){ fields[fc++]=t; t=strtok(NULL,","); }
        if (fc>=5){
            char *node_mac=fields[4]; uint8_t target[6];
            if (parse_mac_string(node_mac,target)){
                bool seen=false; for (int i=0;i<alerted_count;i++){ if (!memcmp(alerted_nodes[i],target,6)){ seen=true; break; } }
                if (!seen){
                    send_alert_notification(target,150.0f); alerts_sent++;
                    if (alerted_count<5){ memcpy(alerted_nodes[alerted_count],target,6); alerted_count++; }
                }
            }
        }
    }
    if (alerts_sent>0) ESP_LOGI(TAG,"Sent %d alert(s)", alerts_sent);
}

static esp_err_t poll_alerts_from_influxdb(void) {
    char url[512]; snprintf(url,sizeof(url), "%s/api/v2/query?org=%s", INFLUXDB_URL, INFLUXDB_ORG);
    char query[512]; snprintf(query,sizeof(query),
        "from(bucket:\"%s\") |> range(start: -10s) |> filter(fn: (r) => r._measurement == \"alert\" and r._field == \"active\" and r._value == true) |> last() |> keep(columns: [\"_time\", \"node\", \"_value\"]) |> yield()",
        ALERTS_DB);

    esp_http_client_config_t cfg={.url=url,.method=HTTP_METHOD_POST,.timeout_ms=10000,.disable_auto_redirect=false,.crt_bundle_attach=esp_crt_bundle_attach,.buffer_size=4096};
    esp_http_client_handle_t c=esp_http_client_init(&cfg);
    if (!c) { ESP_LOGE(TAG,"HTTP client failed"); return ESP_FAIL; }

    char auth[300]; snprintf(auth,sizeof(auth),"Token %s", ALERTS_INFLUXDB_TOKEN);
    esp_http_client_set_header(c,"Authorization",auth);
    esp_http_client_set_header(c,"Content-Type","application/vnd.flux");
    esp_http_client_set_header(c,"Accept","application/csv");
    esp_http_client_set_header(c,"Accept-Encoding","identity");

    esp_err_t err=esp_http_client_open(c, strlen(query));
    if (err!=ESP_OK){ ESP_LOGE(TAG,"open: %s", esp_err_to_name(err)); esp_http_client_cleanup(c); return err; }
    int w=esp_http_client_write(c, query, strlen(query));
    if (w<=0){ ESP_LOGE(TAG,"write failed"); esp_http_client_cleanup(c); return ESP_FAIL; }

    esp_http_client_fetch_headers(c);
    int code=esp_http_client_get_status_code(c);
    char resp[4096]={0}; int total=0, rd;
    while ((rd=esp_http_client_read(c, resp+total, sizeof(resp)-total-1))>0){ total+=rd; }
    resp[total]='\0';

    if (code==200 && total>0) {
        if (strstr(resp,"true")) process_alert_response(resp,total);
    } else if (code!=200) {
        ESP_LOGE(TAG,"HTTP %d", code);
    }
    esp_http_client_cleanup(c);
    return ESP_OK;
}

// ========= Wi-Fi (simplified) =========
// Put this anywhere near the top of your file
static void wifi_reconnect_task(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // wait 5s
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            // already connected — stop retrying
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

    esp_wifi_deinit();  // fully release driver
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
    esp_wifi_set_ps(WIFI_PS_NONE); // keep stable
}


static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 

    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(app_events, EVT_WIFI_OK);
        ESP_LOGI(TAG, "Got IP — Wi-Fi connected successfully");

        // ---- Auto-promotion logic ----
        if (role == ROLE_CHILD || role == ROLE_ISOLATED) {
            ESP_LOGI(TAG, "Child/isolated node connected to router — promoting to ROOT");
            role = ROLE_ROOT;
            memcpy(root_mac, my_mac, 6);
            memcpy(parent_mac, my_mac, 6);
            parent_link_up = true;
            current_layer = 0;
            led_root();

            // Start root-specific tasks if not already running
            xTaskCreate(root_beacon_task, "root_beacon", 4096, NULL, 5, NULL);
            if (!data_timer_running) init_data_timer();
            if (!alert_timer_running) init_alert_timer();
            initialize_sntp();

            ESP_LOGI(TAG, "Promoted to ROOT and beaconing...");
        }
    } 

    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (role == ROLE_ROOT) {
            // Root lost router connection — clean reboot to recover Wi-Fi properly
            ESP_LOGW(TAG, "Root lost Wi-Fi — restarting to re-establish router link");
            esp_restart();
        } else {
            // Child node lost router connection — fallback to mesh mode but keep retrying
            ESP_LOGW(TAG, "Child lost Wi-Fi — staying in mesh mode and retrying router every 5s");

            // Spawn (or reuse) a reconnect task that runs in the background
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
    sta.sta.pmf_cfg.capable = false;  // make compatible with all APs
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


// ========= ESP-NOW =========
static void espnow_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    if (!mac) return;
    static int parent_fail_count = 0;
    if (status == ESP_NOW_SEND_SUCCESS) {
        if (!mac_equal(mac,(uint8_t[6]){0xFF,0xFF,0xFF,0xFF,0xFF,0xFF})) {
            parent_link_up = true;
            last_parent_seen_us = now_us();
            parent_fail_count = 0;
        }
    } else {
        if (!mac_is_zero(parent_mac) && mac_equal(mac,parent_mac)) {
            parent_fail_count++;
            if (parent_fail_count >= 5) {
                restart_node("Parent send failures");
            }
        }
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data || len < (int)sizeof(mesh_hdr_t)) return;
    int8_t rssi = info->rx_ctrl->rssi;
    const uint8_t *from = info->src_addr;
    mesh_hdr_t *hdr = (mesh_hdr_t*)data; hdr->rssi = rssi;

    switch (hdr->type) {
        case PKT_BEACON: {
            if (len < (int)sizeof(beacon_pkt_t)) break;
            const beacon_pkt_t *b = (const beacon_pkt_t*)data;

            if (role != ROLE_ROOT) {
                if (parent_link_up && mac_equal(from, parent_mac)) {
                    last_parent_seen_us = now_us();
                    if (rssi > best_beacon_rssi) best_beacon_rssi = rssi;
                    break;
                }
                bool better = (!parent_link_up) || (rssi > (best_beacon_rssi + 5));
                if (better) {
                    best_beacon_rssi = rssi;
                    memcpy(root_mac, b->root_mac, 6);
                    memcpy(parent_mac, from, 6);
                    current_channel = b->channel;

                    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    ensure_peer(parent_mac, current_channel);

                    join_req_pkt_t rq={0}; rq.hdr.type=PKT_JOIN_REQUEST; rq.hdr.max_hops=5;
                    memcpy(rq.child_mac, my_mac, 6);
                    reliable_send(parent_mac, &rq, sizeof(rq), SEND_RETRY_LIMIT);
                }
            }
            break;
        }
        case PKT_JOIN_REQUEST: {
            if (len < (int)sizeof(join_req_pkt_t)) break;
            if (role == ROLE_ROOT || role == ROLE_CHILD) {
                ensure_peer(from, current_channel);
                join_acc_pkt_t ac={0}; ac.hdr.type=PKT_JOIN_ACCEPT; ac.hdr.max_hops=5;
                memcpy(ac.parent_mac, my_mac, 6); ac.channel=current_channel; ac.layer=current_layer+1;
                reliable_send(from, &ac, sizeof(ac), SEND_RETRY_LIMIT);
            }
            break;
        }
        case PKT_JOIN_ACCEPT: {
            if (role != ROLE_ROOT) {
                const join_acc_pkt_t *ac = (const join_acc_pkt_t*)data;
                if (role==ROLE_CHILD && mac_equal(parent_mac,ac->parent_mac)) { last_parent_seen_us = now_us(); break; }
                memcpy(parent_mac, ac->parent_mac, 6);
                current_channel = ac->channel; current_layer = ac->layer;
                ensure_peer(parent_mac, current_channel);
                parent_link_up = true; last_parent_seen_us = now_us(); role = ROLE_CHILD;
                led_child();
            }
            break;
        }
        case PKT_DATA: {
            if (len < (int)sizeof(data_pkt_t)) break;
            const data_pkt_t *dp = (const data_pkt_t*)data; last_parent_seen_us = now_us();

            if (role == ROLE_ROOT) {
                char src[18]; mac_to_str(dp->src_mac, src, sizeof(src));
                int sensor_value=0, temperature=0;
                if (sscanf((char*)dp->payload,"Node:%*s SENSOR:%d TEMP:%dC",&sensor_value,&temperature)==2) {
                    queue_influxdb_data(src, sensor_value, temperature, dp->hdr.rssi, dp->hdr.hop_count);
                }
                blink_orange(3);
            } else if (parent_link_up && dp->hdr.hop_count < dp->hdr.max_hops) {
                data_pkt_t fwd=*dp; fwd.hdr.hop_count++;
                reliable_send(parent_mac,&fwd,sizeof(fwd),SEND_RETRY_LIMIT);
            }
            break;
        }
        case PKT_ALERT_NOTIFY: {
            if (len < (int)sizeof(alert_notify_pkt_t)) break;
            const alert_notify_pkt_t *al = (const alert_notify_pkt_t*)data;
            if (mac_equal(al->target_mac, my_mac)) {
                alert_led_effect();
                if (parent_link_up && al->hdr.hop_count < al->hdr.max_hops) {
                    alert_notify_pkt_t fwd=*al; fwd.hdr.hop_count++;
                    reliable_send(parent_mac, &fwd, sizeof(fwd), SEND_RETRY_LIMIT);
                }
            }
            break;
        }
        default: break;
    }
}

// ========= Root Beacon =========
static void root_beacon_task(void *arg) {
    uint8_t bmac[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    setup_broadcast_peer(current_channel);
    while (1) {
        if (role != ROLE_ROOT) vTaskDelete(NULL);
        uint8_t pri=0; wifi_second_chan_t sec=WIFI_SECOND_CHAN_NONE; esp_wifi_get_channel(&pri,&sec);
        if (pri && pri!=current_channel){ current_channel=pri; setup_broadcast_peer(current_channel); }
        beacon_pkt_t b={0}; b.hdr.type=PKT_BEACON; b.hdr.max_hops=8;
        memcpy(b.root_mac,my_mac,6); memcpy(b.parent_mac,my_mac,6); b.channel=current_channel; b.layer=0;
        esp_now_send(bmac,(uint8_t*)&b,sizeof(b));
        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
    }
}

// ========= Child Task (parent-loss -> restart; self-promote to root) =========
static void child_task(void *arg) {
    setup_broadcast_peer(current_channel);
    int scan_attempts=0; int64_t last_root_attempt_us=0;
    const int64_t ROOT_ATTEMPT_INTERVAL_US = 15000000; // 15s

    while (1) {
        int64_t n = now_us();

        // Parent timeout => restart fast
        if (parent_link_up && (n - last_parent_seen_us) > (int64_t)PARENT_LOSS_MS*1000) {
            restart_node("Parent timeout");
        }
        if (!parent_link_up && !mac_is_zero(parent_mac) && (n - last_parent_seen_us) > (int64_t)PARENT_LOSS_MS*2000) {
            restart_node("Stale parent state");
        }

        // Try to become root every 15s if isolated or link is down
        if ((role == ROLE_ISOLATED || !parent_link_up) && (n - last_root_attempt_us) > ROOT_ATTEMPT_INTERVAL_US) {
            last_root_attempt_us = n;
            esp_wifi_disconnect(); vTaskDelay(pdMS_TO_TICKS(400)); esp_wifi_connect();
            EventBits_t bits = xEventGroupWaitBits(app_events, EVT_WIFI_OK, pdFALSE, pdFALSE, pdMS_TO_TICKS(6000));
            if (bits & EVT_WIFI_OK) {
                wifi_ap_record_t ap;
                if (esp_wifi_sta_get_ap_info(&ap)==ESP_OK) {
                    uint8_t new_ch=ap.primary;
                    role = ROLE_ROOT; current_channel=new_ch; memcpy(root_mac,my_mac,6); current_layer=0;
                    parent_link_up=true; memcpy(parent_mac,my_mac,6);

                    xQueueReset(influxdb_queue);
                    for(int i=0;i<3;i++){ set_led(0,0,255); vTaskDelay(pdMS_TO_TICKS(160)); set_led(0,0,0); vTaskDelay(pdMS_TO_TICKS(160)); }
                    led_root();

                    xTaskCreate(root_beacon_task,"root_beacon",4096,NULL,5,NULL);
                    if (!data_timer_running) init_data_timer();
                    if (!alert_timer_running) init_alert_timer();
                    initialize_sntp();

                    vTaskDelete(NULL);
                    return;
                }
            }
        }

        // Scan hopping while isolated
        if (role == ROLE_ISOLATED || !parent_link_up) {
            scan_attempts++;
            if (scan_attempts % 30 == 0) {
                uint8_t new_ch = current_channel + 1; if (new_ch>13) new_ch=1;
                current_channel=new_ch; esp_wifi_set_channel(current_channel,WIFI_SECOND_CHAN_NONE);
                vTaskDelay(pdMS_TO_TICKS(30)); setup_broadcast_peer(current_channel);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ========= Init Bits =========
static void init_led_strip(void) {
    led_strip_config_t sc={.strip_gpio_num=LED_PIN,.max_leds=1,.led_model=LED_MODEL_WS2812};
    led_strip_rmt_config_t rc={.resolution_hz=10*1000*1000,.flags.with_dma=false};
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc,&rc,&led_strip));
    set_led(0,0,0);
}
static void init_button(void) {
    button_event_queue = xQueueCreate(10,sizeof(button_event_t));
    gpio_config_t io={.pin_bit_mask=(1ULL<<BOOT_BUTTON_PIN),.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_NEGEDGE};
    gpio_config(&io); gpio_install_isr_service(0); gpio_isr_handler_add(BOOT_BUTTON_PIN, button_isr_handler, NULL);
    xTaskCreate(button_task,"button_task",4096,NULL,10,NULL);
}
static void initialize_sntp(void) {
    static bool sntp_started = false;   // prevent double-start
    if (sntp_started) {
        ESP_LOGW(TAG, "SNTP already running — skipping reinit");
        return;
    }

    ESP_LOGI(TAG, "SNTP init");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
#else
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
#endif

    sntp_started = true;

    // Wait for time sync once
    time_t now = 0;
    struct tm ti = {0};
    int retry = 0;
    while (ti.tm_year < (2016 - 1900) && ++retry < 15) {
        vTaskDelay(pdMS_TO_TICKS(800));
        time(&now);
        localtime_r(&now, &ti);
    }

    if (ti.tm_year >= (2016 - 1900))
        ESP_LOGI(TAG, "SNTP time synchronized");
    else
        ESP_LOGW(TAG, "SNTP time sync timeout");
}


static void process_auto_send(void) {
    if (role == ROLE_ROOT) {
        float v = turbidity_read_voltage();
        if (!turbidity_sensor_present) { ESP_LOGW(TAG,"No turbidity — skip root autosend"); return; }
        int st = turbidity_get_status(v); int sv=(int)(v*100.0f); int t=25;
        const char *state=(st==0)?"CLEAR":(st==1)?"CLOUDY":"DIRTY";
        char mac_str[18]; mac_to_str(my_mac, mac_str, sizeof(mac_str));
        ESP_LOGI(TAG, "ROOT auto-send: %.2fV (%s) -> SENSOR:%d TEMP:%dC", v,state,sv,t);
        queue_influxdb_data(mac_str, sv, t, -65, 0);
        set_led(255,255,255); vTaskDelay(pdMS_TO_TICKS(40)); led_root();
    } else {
        send_data_packet();
    }
}

// ========= MAIN =========
void app_main(void) {
    ESP_LOGI(TAG, "Booting mesh hybrid (clean restart-on-loss)");

    init_led_strip();
    init_button();
    turbidity_init();

    float vcheck = turbidity_read_voltage();
    ESP_LOGI(TAG,"Initial turbidity: %.2fV", vcheck);
    if (vcheck < 1.0f && vcheck > 0.55f) { turbidity_sensor_present=false; ESP_LOGW(TAG,"No turbidity sensor detected — continuing"); }
    else { turbidity_sensor_present=true; }

    pool_mutex = xSemaphoreCreateMutex();
    influxdb_queue = xQueueCreate(30, sizeof(influxdb_data_t*));
    xTaskCreate(influxdb_task, "influxdb_task", 16384, NULL, 4, NULL);

    esp_err_t ret = nvs_flash_init();
    if (ret==ESP_ERR_NVS_NO_FREE_PAGES || ret==ESP_ERR_NVS_NEW_VERSION_FOUND){ ESP_ERROR_CHECK(nvs_flash_erase()); ESP_ERROR_CHECK(nvs_flash_init()); }

    app_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_read_mac(my_mac, ESP_MAC_WIFI_STA));
    char my[18]; mac_to_str(my_mac,my,sizeof(my)); ESP_LOGI(TAG,"MAC %s", my);

    bool joined=false; uint8_t ap_ch=1;
    ESP_ERROR_CHECK(wifi_init_sta_or_child(&joined, &ap_ch));
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESPNOW_PMK));
    ESP_ERROR_CHECK(esp_now_register_send_cb((esp_now_send_cb_t)espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb((esp_now_recv_cb_t)espnow_recv_cb));

    if (joined) {
        role=ROLE_ROOT; memcpy(root_mac,my_mac,6); current_channel=ap_ch; current_layer=0; parent_link_up=true; memcpy(parent_mac,my_mac,6);
        led_root();
        initialize_sntp();
        xTaskCreate(root_beacon_task,"root_beacon",4096,NULL,5,NULL);
        init_alert_timer();
        ESP_LOGI(TAG,"Root ready");
    } else {
        role=ROLE_ISOLATED; current_channel=1; esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
        led_isolated();
        xTaskCreate(child_task,"child_task",4096,NULL,5,NULL);
        ESP_LOGI(TAG,"Child scanning");
    }

    init_data_timer();

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(app_events, EVT_AUTO_SEND | EVT_POLL_ALERTS, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & EVT_AUTO_SEND)  process_auto_send();
        if ((bits & EVT_POLL_ALERTS) && role==ROLE_ROOT) poll_alerts_from_influxdb();
    }
}
