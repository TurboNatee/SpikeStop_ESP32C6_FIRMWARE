#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include "driver/gpio.h"
#include "freertos/queue.h"
#include "esp_timer.h"

int64_t now_us(void);
void provisioning_toggle_share_mode(void);
void provisioning_factory_reset(void);

typedef enum {
    BUTTON_SHORT_PRESS,
    BUTTON_LONG_PRESS
} button_event_t;

static QueueHandle_t button_event_queue = NULL;
static volatile int64_t last_button_press_us = 0;

static void check_factory_reset_hold_on_boot(void) {
    if (gpio_get_level(BOOT_BUTTON_PIN) != 0) {
        return;
    }

    ESP_LOGW(TAG, "BOOT held at startup; hold for 5s to factory reset");
    int64_t start = now_us();
    bool threshold_reached = false;
    while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (!threshold_reached && (now_us() - start) >= (BUTTON_FACTORY_RESET_MS * 1000LL)) {
            threshold_reached = true;
            ESP_LOGW(TAG, "Startup hold threshold reached; release BOOT to confirm factory reset");
        }
    }

    if (threshold_reached) {
        provisioning_factory_reset();
    } else {
        ESP_LOGI(TAG, "Startup hold canceled before factory-reset threshold");
    }
}

static void IRAM_ATTR button_isr_handler(void *arg) {
    int64_t n = now_us();
    if ((n - last_button_press_us) > BUTTON_DEBOUNCE_MS * 1000) {
        last_button_press_us = n;
        button_event_t e = BUTTON_SHORT_PRESS;
        xQueueSendFromISR(button_event_queue, &e, NULL);
    }
}

static void button_task(void *arg) {
    button_event_t e;
    int64_t start = 0;
    bool pressed = false;

    while (1) {
        if (xQueueReceive(button_event_queue, &e, portMAX_DELAY) == pdTRUE) {
            if (e == BUTTON_SHORT_PRESS) {
                start = now_us();
                pressed = true;
                while (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if ((now_us() - start) > BUTTON_FACTORY_RESET_MS * 1000) {
                        break;
                    }
                }
                if (pressed) {
                    pressed = false;
                    int64_t press_us = now_us() - start;
                    if (press_us >= BUTTON_FACTORY_RESET_MS * 1000) {
                        provisioning_factory_reset();
                    } else if (press_us >= BUTTON_LONG_PRESS_MS * 1000) {
                        provisioning_toggle_share_mode();
                    } else {
                        ESP_LOGI(TAG, "Short press ignored");
                    }
                }
            }
        }
    }
}

static void init_button(void) {
    button_event_queue = xQueueCreate(10, sizeof(button_event_t));
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io);
    check_factory_reset_hold_on_boot();
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_PIN, button_isr_handler, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
}

#endif 
