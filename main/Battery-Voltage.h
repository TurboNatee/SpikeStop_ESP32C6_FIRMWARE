#ifndef BATTERY_VOLTAGE_H
#define BATTERY_VOLTAGE_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifndef BATTERY_ADC_GPIO
#define BATTERY_ADC_GPIO 5
#endif

#ifndef BATTERY_DIVIDER_RATIO
#define BATTERY_DIVIDER_RATIO 2.0f
#endif

static adc_oneshot_unit_handle_t adc_get_or_init_unit1(void);

static adc_oneshot_unit_handle_t battery_adc_handle;
static adc_channel_t battery_adc_channel;
static bool battery_adc_ready = false;
static adc_cali_handle_t battery_adc_cali = NULL;
static bool battery_adc_cali_ready = false;

static void battery_voltage_init(void) {
    adc_unit_t unit = ADC_UNIT_1;
    if (adc_oneshot_io_to_channel(BATTERY_ADC_GPIO, &unit, &battery_adc_channel) != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "Battery ADC GPIO%d is not ADC1-capable", BATTERY_ADC_GPIO);
        battery_adc_ready = false;
        return;
    }

    battery_adc_handle = adc_get_or_init_unit1();
    if (battery_adc_handle == NULL) {
        ESP_LOGE(TAG, "Battery ADC could not acquire ADC1");
        battery_adc_ready = false;
        return;
    }

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(battery_adc_handle, battery_adc_channel, &ch_cfg));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = battery_adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &battery_adc_cali) == ESP_OK) {
        battery_adc_cali_ready = true;
    }
#endif

    battery_adc_ready = true;
    ESP_LOGI(TAG, "Battery ADC init @ GPIO%d (divider x%.2f, cali:%s)",
             BATTERY_ADC_GPIO,
             BATTERY_DIVIDER_RATIO,
             battery_adc_cali_ready ? "on" : "off");
}

static float battery_read_voltage(void) {
    if (!battery_adc_ready) {
        return NAN;
    }

    const int samples = 8;
    int acc = 0;
    for (int i = 0; i < samples; i++) {
        int raw = 0;
        if (adc_oneshot_read(battery_adc_handle, battery_adc_channel, &raw) != ESP_OK) {
            return NAN;
        }
        acc += raw;
    }

    int raw_avg = acc / samples;
    float v_adc;
    if (battery_adc_cali_ready) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(battery_adc_cali, raw_avg, &mv) != ESP_OK) {
            return NAN;
        }
        v_adc = (float)mv / 1000.0f;
    } else {
        v_adc = ((float)raw_avg / 4095.0f) * 3.3f;
    }

    return v_adc * BATTERY_DIVIDER_RATIO;
}

static int battery_estimate_percent(float vbat) {
    if (isnan(vbat)) {
        return -1;
    }
    if (vbat < 0.20f) return -1;
    if (vbat <= 3.20f) return 0;
    if (vbat <= 3.40f) return 10;
    if (vbat <= 3.65f) return 25;
    if (vbat <= 3.80f) return 50;
    if (vbat <= 3.95f) return 75;
    if (vbat <= 4.20f) return 100;
    return 100;
}

#endif
