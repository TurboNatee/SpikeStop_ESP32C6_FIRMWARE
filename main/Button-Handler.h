#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include "driver/gpio.h"
#include "freertos/queue.h"
#include "esp_timer.h"

// Forward declarations
void send_sample_data_packet(void);
int64_t now_us(void);

// Button event types
typedef enum { BUTTON_SHORT_PRESS, BUTTON_LONG_PRESS } button_event_t;

static QueueHandle_t button_event_queue = NULL;
static volatile int64_t last_button_press_us = 0;

static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t n = now_us();
    if ((n - last_button_press_us) > BUTTON_DEBOUNCE_MS * 1000) {
        last_button_press_us = n;
        button_event_t e = BUTTON_SHORT_PRESS;
        xQueueSendFromISR(button_event_queue, &e, NULL);
    }
}

static void button_task(void* arg) {
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
                    if ((now_us() - start) > BUTTON_LONG_PRESS_MS * 1000) break;
                }
                if (pressed) {
                    pressed = false;
                    if ((now_us() - start) < BUTTON_LONG_PRESS_MS * 1000) {
                        send_sample_data_packet();
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
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_PIN, button_isr_handler, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
}

#endif // BUTTON_HANDLER_H
