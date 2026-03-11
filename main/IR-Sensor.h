#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

#ifndef IR_PHOTODIODE_GPIO
#define IR_PHOTODIODE_GPIO 2
#endif

#ifndef IR_EMITTER_GPIO
#define IR_EMITTER_GPIO 3
#endif

#ifndef IR_SAMPLE_INTERVAL_MS
#define IR_SAMPLE_INTERVAL_MS 500
#endif

#ifndef IR_BREAK_THRESHOLD_MV
#define IR_BREAK_THRESHOLD_MV 120
#endif

static adc_oneshot_unit_handle_t adc_get_or_init_unit1(void);

static adc_oneshot_unit_handle_t ir_adc_handle;
static adc_channel_t ir_adc_channel;
static bool ir_sensor_ready = false;
static volatile int ir_last_signal_mv = 0;
static volatile bool ir_last_broken = true;
static volatile bool ir_last_valid = false;

static bool ir_get_last_signal_mv(int *mv_out) {
    if (mv_out == NULL || !ir_last_valid) {
        return false;
    }
    *mv_out = ir_last_signal_mv;
    return true;
}

static bool ir_get_last_broken(bool *broken_out) {
    if (broken_out == NULL || !ir_last_valid) {
        return false;
    }
    *broken_out = ir_last_broken;
    return true;
}

static void ir_emitter_set(bool on) {
    gpio_set_level((gpio_num_t)IR_EMITTER_GPIO, on ? 1 : 0);
}

static bool ir_read_avg_mv(int *mv_out) {
    if (!ir_sensor_ready || mv_out == NULL) {
        return false;
    }

    const int samples = 6;
    int acc = 0;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        if (adc_oneshot_read(ir_adc_handle, ir_adc_channel, &raw) != ESP_OK) {
            return false;
        }
        acc += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    int raw_avg = acc / samples;
    *mv_out = (int)(((float)raw_avg / 4095.0f) * 3300.0f);
    return true;
}

static void ir_sensor_task(void *arg) {
    bool last_broken = false;
    bool first = true;

    while (1) {
        int ambient_mv = 0;
        int lit_mv = 0;

        ir_emitter_set(false);
        vTaskDelay(pdMS_TO_TICKS(2));
        bool ok_ambient = ir_read_avg_mv(&ambient_mv);

        ir_emitter_set(true);
        vTaskDelay(pdMS_TO_TICKS(2));
        bool ok_lit = ir_read_avg_mv(&lit_mv);

        int signal_mv = ambient_mv - lit_mv;
        bool broken = signal_mv < IR_BREAK_THRESHOLD_MV;

        if (!ok_ambient || !ok_lit) {
            ESP_LOGW(TAG, "IR read failed");
        } else {
            ir_last_signal_mv = signal_mv;
            ir_last_broken = broken;
            ir_last_valid = true;

            ESP_LOGI(TAG,
                     "IR raw: amb=%dmV lit=%dmV signal=%dmV thr=%dmV -> %s",
                     ambient_mv,
                     lit_mv,
                     signal_mv,
                     IR_BREAK_THRESHOLD_MV,
                     broken ? "BROKEN" : "CLEAR");

            if (first || broken != last_broken) {
                ESP_LOGI(TAG, "IR beam %s", broken ? "BROKEN" : "CLEAR");
                last_broken = broken;
                first = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(IR_SAMPLE_INTERVAL_MS));
    }
}

static void ir_sensor_init(void) {
    gpio_config_t out_cfg = {
        .pin_bit_mask = 1ULL << IR_EMITTER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    ir_emitter_set(false);

    adc_unit_t unit = ADC_UNIT_1;
    if (adc_oneshot_io_to_channel(IR_PHOTODIODE_GPIO, &unit, &ir_adc_channel) != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "IR photodiode GPIO%d is not ADC1-capable", IR_PHOTODIODE_GPIO);
        ir_sensor_ready = false;
        return;
    }

    ir_adc_handle = adc_get_or_init_unit1();
    if (ir_adc_handle == NULL) {
        ESP_LOGE(TAG, "IR could not acquire ADC1");
        ir_sensor_ready = false;
        return;
    }

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(ir_adc_handle, ir_adc_channel, &ch_cfg));

    ir_sensor_ready = true;
    ESP_LOGI(TAG,
             "IR sensor init: photodiode GPIO%d, emitter GPIO%d, interval=%dms, threshold=%dmV",
             IR_PHOTODIODE_GPIO,
             IR_EMITTER_GPIO,
             IR_SAMPLE_INTERVAL_MS,
             IR_BREAK_THRESHOLD_MV);

    xTaskCreate(ir_sensor_task, "ir_sensor_task", 3072, NULL, 3, NULL);
}

#endif
