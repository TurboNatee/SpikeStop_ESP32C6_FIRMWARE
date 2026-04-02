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

#ifndef IR_READ_SAMPLES
#define IR_READ_SAMPLES 24
#endif

#ifndef IR_DYNAMIC_BREAK_DROP_MV
#define IR_DYNAMIC_BREAK_DROP_MV 2
#endif

#ifndef IR_DYNAMIC_CLEAR_DROP_MV
#define IR_DYNAMIC_CLEAR_DROP_MV 1
#endif

#ifndef IR_REPORT_GAIN_X1000
#define IR_REPORT_GAIN_X1000 1000
#endif

static adc_oneshot_unit_handle_t adc_get_or_init_unit1(void);

static adc_oneshot_unit_handle_t ir_adc_handle;
static adc_channel_t ir_adc_channel;
static bool ir_sensor_ready = false;
static volatile int ir_last_signal_mv = 0;
static volatile int32_t ir_last_signal_uv = 0;
static volatile bool ir_last_broken = true;
static volatile bool ir_last_valid = false;
static int32_t ir_filtered_signal_uv = 0;
static int32_t ir_baseline_signal_uv = 0;
static bool ir_filter_ready = false;

static int32_t ir_apply_report_gain_uv(int32_t signal_uv) {
    int64_t scaled = ((int64_t)signal_uv * (int64_t)IR_REPORT_GAIN_X1000) / 1000;
    if (scaled > INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled < 0) {
        return 0;
    }
    return (int32_t)scaled;
}

static bool ir_get_last_signal_mv(int *mv_out) {
    if (mv_out == NULL || !ir_last_valid) {
        return false;
    }
    *mv_out = ir_last_signal_mv;
    return true;
}

static bool ir_get_last_signal_uv(int32_t *uv_out) {
    if (uv_out == NULL || !ir_last_valid) {
        return false;
    }
    *uv_out = ir_last_signal_uv;
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

static bool ir_read_avg_uv(int32_t *uv_out) {
    if (!ir_sensor_ready || uv_out == NULL) {
        return false;
    }

    const int samples = IR_READ_SAMPLES;
    int acc = 0;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        if (adc_oneshot_read(ir_adc_handle, ir_adc_channel, &raw) != ESP_OK) {
            return false;
        }
        acc += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    int32_t denom = (int32_t)samples * 4095;
    int32_t uv = (int32_t)(((int64_t)acc * 3300000LL) / denom);
    *uv_out = uv;
    return true;
}

static void ir_sensor_task(void *arg) {
    bool last_broken = false;
    bool first = true;

    while (1) {
        int32_t ambient_uv = 0;
        int32_t lit_uv = 0;

        ir_emitter_set(false);
        vTaskDelay(pdMS_TO_TICKS(2));
        bool ok_ambient = ir_read_avg_uv(&ambient_uv);

        ir_emitter_set(true);
        vTaskDelay(pdMS_TO_TICKS(2));
        bool ok_lit = ir_read_avg_uv(&lit_uv);

        int32_t signal_uv = ambient_uv - lit_uv;
        if (signal_uv < 0) {
            signal_uv = -signal_uv;
        }

        if (!ok_ambient || !ok_lit) {
            ESP_LOGW(TAG, "IR read failed");
        } else {
            if (!ir_filter_ready) {
                ir_filtered_signal_uv = signal_uv;
                ir_baseline_signal_uv = signal_uv;
                ir_filter_ready = true;
            } else {
                ir_filtered_signal_uv = (ir_filtered_signal_uv * 3 + signal_uv) / 4;

                if (ir_filtered_signal_uv > ir_baseline_signal_uv) {
                    ir_baseline_signal_uv = ir_filtered_signal_uv;
                } else if (!ir_last_broken && ir_baseline_signal_uv > ir_filtered_signal_uv) {
                    ir_baseline_signal_uv -= 1000;
                    if (ir_baseline_signal_uv < ir_filtered_signal_uv) {
                        ir_baseline_signal_uv = ir_filtered_signal_uv;
                    }
                }
            }

            int32_t drop_uv = ir_baseline_signal_uv - ir_filtered_signal_uv;
            if (drop_uv < 0) {
                drop_uv = 0;
            }

            int32_t break_thr_uv = (int32_t)IR_BREAK_THRESHOLD_MV * 1000;
            int32_t dynamic_break_drop_uv = (int32_t)IR_DYNAMIC_BREAK_DROP_MV * 1000;
            int32_t dynamic_clear_drop_uv = (int32_t)IR_DYNAMIC_CLEAR_DROP_MV * 1000;

            bool broken;
            if (first) {
                broken = (ir_filtered_signal_uv < break_thr_uv) ||
                         (drop_uv >= dynamic_break_drop_uv);
            } else if (last_broken) {
                broken = (ir_filtered_signal_uv < break_thr_uv) ||
                         (drop_uv > dynamic_clear_drop_uv);
            } else {
                broken = (ir_filtered_signal_uv < break_thr_uv) ||
                         (drop_uv >= dynamic_break_drop_uv);
            }

            int32_t signal_uv_report = ir_apply_report_gain_uv(signal_uv);
            ir_last_signal_uv = signal_uv_report;
            ir_last_signal_mv = (int)((signal_uv_report + 500) / 1000);
            ir_last_broken = broken;
            ir_last_valid = true;

            ESP_LOGI(TAG,
                     "IR raw: amb=%d.%03dmV lit=%d.%03dmV signal=%d.%03dmV filt=%d.%03dmV base=%d.%03dmV drop=%d.%03dmV thr=%dmV -> %s",
                     (int)(ambient_uv / 1000),
                     (int)(ambient_uv % 1000),
                     (int)(lit_uv / 1000),
                     (int)(lit_uv % 1000),
                     (int)(signal_uv / 1000),
                     (int)(signal_uv % 1000),
                     (int)(ir_filtered_signal_uv / 1000),
                     (int)(ir_filtered_signal_uv % 1000),
                     (int)(ir_baseline_signal_uv / 1000),
                     (int)(ir_baseline_signal_uv % 1000),
                     (int)(drop_uv / 1000),
                     (int)(drop_uv % 1000),
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
             "IR sensor init: photodiode GPIO%d, emitter GPIO%d, interval=%dms, threshold=%dmV, samples=%d",
             IR_PHOTODIODE_GPIO,
             IR_EMITTER_GPIO,
             IR_SAMPLE_INTERVAL_MS,
             IR_BREAK_THRESHOLD_MV,
             IR_READ_SAMPLES);

    xTaskCreate(ir_sensor_task, "ir_sensor_task", 3072, NULL, 3, NULL);
}

#endif
