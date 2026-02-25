#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "esp_log.h"
#include "driver/gpio.h"
#include "led_strip.h"

static led_strip_handle_t led_strip;
static bool alert_active = false;

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

static void led_root(void) {
    set_led(0, 0, 255);
}

static void led_child(void) {
    set_led(0, 255, 0);
}

static void led_isolated(void) {
    set_led(255, 0, 0);
}

static void blink_orange(int times) {
    for (int i = 0; i < times; i++) {
        set_led(255, 165, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        set_led(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    if (role == ROLE_ROOT) {
        led_root();
    } else if (role == ROLE_CHILD) {
        led_child();
    } else {
        led_isolated();
    }
}

void alert_led_task(void *pvParameters) {
    while (1) {
        if (alert_active) {
            set_led(255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
            set_led(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void alert_led_effect(void) {
    alert_active = true;
}

static void init_led_strip(void) {
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
