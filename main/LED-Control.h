#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_timer.h"

static led_strip_handle_t led_strip;
static bool alert_active = false;
static int64_t status_led_until_us = 0;
static uint8_t status_led_r = 0;
static uint8_t status_led_g = 0;
static uint8_t status_led_b = 0;
static bool status_led_solid = false;

#ifndef ALERT_LED_GPIO
#define ALERT_LED_GPIO 4
#endif

#ifndef ALERT_LED_ACTIVE_LEVEL
#define ALERT_LED_ACTIVE_LEVEL 1
#endif

static void set_alert_led(bool on) {
    int level = on ? ALERT_LED_ACTIVE_LEVEL : !ALERT_LED_ACTIVE_LEVEL;
    gpio_set_level((gpio_num_t)ALERT_LED_GPIO, level);
}

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

static void apply_status_led(void) {
    if (status_led_solid) {
        set_led(status_led_r, status_led_g, status_led_b);
        return;
    }

    if (status_led_until_us > esp_timer_get_time()) {
        set_led(status_led_r, status_led_g, status_led_b);
    } else {
        set_led(0, 0, 0);
    }
}

static void set_temporary_status_led(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms) {
    status_led_r = r;
    status_led_g = g;
    status_led_b = b;
    status_led_solid = false;
    status_led_until_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000LL);
    set_led(r, g, b);
}

static void led_root(void) {
    set_temporary_status_led(0, 0, 255, 5000);
}

static void led_child(void) {
    set_temporary_status_led(0, 255, 0, 5000);
}

static void led_isolated(void) {
    status_led_r = 255;
    status_led_g = 0;
    status_led_b = 0;
    status_led_solid = true;
    status_led_until_us = 0;
    set_led(255, 0, 0);
}

static void blink_orange(int times) {
    for (int i = 0; i < times; i++) {
        set_led(255, 165, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        set_led(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    apply_status_led();
}

void alert_led_task(void *pvParameters) {
    while (1) {
        if (alert_active) {
            set_alert_led(true);
            vTaskDelay(pdMS_TO_TICKS(300));
            set_alert_led(false);
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            set_alert_led(false);
            apply_status_led();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void alert_led_effect(void) {
    alert_active = true;
}

static void init_led_strip(void) {
    gpio_config_t alert_io = {
        .pin_bit_mask = 1ULL << ALERT_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&alert_io));
    set_alert_led(false);
    ESP_LOGI(TAG, "External alert LED init @ GPIO%d (active=%d)", ALERT_LED_GPIO, ALERT_LED_ACTIVE_LEVEL);

    for (int i = 0; i < 2; i++) {
        set_alert_led(true);
        vTaskDelay(pdMS_TO_TICKS(120));
        set_alert_led(false);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    led_strip_config_t sc = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812
    };
    led_strip_rmt_config_t rc = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &led_strip));
    set_led(0, 0, 0);
}

#endif 
