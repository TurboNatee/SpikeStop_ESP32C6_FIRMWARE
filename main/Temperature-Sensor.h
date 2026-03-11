#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <math.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifndef TEMP_SENSOR_GPIO
#define TEMP_SENSOR_GPIO 1
#endif

static adc_oneshot_unit_handle_t temperature_adc_handle;
static adc_channel_t temperature_adc_channel;
static bool temperature_sensor_present = false;
static adc_oneshot_unit_handle_t adc_get_or_init_unit1(void);
static adc_cali_handle_t temperature_adc_cali = NULL;
static bool temperature_adc_cali_ready = false;

#ifndef TEMP_SENSOR_NTC_R0_OHM
#define TEMP_SENSOR_NTC_R0_OHM            10000.0f
#endif

#ifndef TEMP_SENSOR_NTC_BETA
#define TEMP_SENSOR_NTC_BETA              3950.0f
#endif

#ifndef TEMP_SENSOR_SERIES_RES_OHM
#define TEMP_SENSOR_SERIES_RES_OHM        10000.0f
#endif

#ifndef TEMP_SENSOR_NTC_TO_VCC
#define TEMP_SENSOR_NTC_TO_VCC            1
#endif

#ifndef TEMP_SENSOR_VREF
#define TEMP_SENSOR_VREF                  3.3f
#endif

#ifndef TEMP_SENSOR_T0_K
#define TEMP_SENSOR_T0_K                  298.15f
#endif

static void temperature_sensor_init(void) {
    adc_unit_t unit = ADC_UNIT_1;
    if (adc_oneshot_io_to_channel(TEMP_SENSOR_GPIO, &unit, &temperature_adc_channel) != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d is not ADC-capable for NTC sensor", TEMP_SENSOR_GPIO);
        temperature_sensor_present = false;
        return;
    }

    if (unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO%d resolved to unsupported ADC unit", TEMP_SENSOR_GPIO);
        temperature_sensor_present = false;
        return;
    }

    temperature_adc_handle = adc_get_or_init_unit1();
    if (temperature_adc_handle == NULL) {
        ESP_LOGE(TAG, "Failed to initialize shared ADC1 for temperature");
        temperature_sensor_present = false;
        return;
    }
    adc_oneshot_chan_cfg_t ch_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(temperature_adc_handle, temperature_adc_channel, &ch_cfg));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = temperature_adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &temperature_adc_cali) == ESP_OK) {
        temperature_adc_cali_ready = true;
    }
#endif

    temperature_sensor_present = true;
    ESP_LOGI(TAG, "NTC10K(B3950) ADC init @ GPIO%d (cali:%s)",
             TEMP_SENSOR_GPIO,
             temperature_adc_cali_ready ? "on" : "off");
}

static float temperature_read_c_internal(bool log_debug) {
    if (!temperature_sensor_present) {
        return NAN;
    }

    int raw_acc = 0;
    const int samples = 8;
    for (int i = 0; i < samples; i++) {
        int raw_i = 0;
        if (adc_oneshot_read(temperature_adc_handle, temperature_adc_channel, &raw_i) != ESP_OK) {
            return NAN;
        }
        raw_acc += raw_i;
    }
    int raw = raw_acc / samples;

    int mv = 0;
    if (temperature_adc_cali_ready) {
        if (adc_cali_raw_to_voltage(temperature_adc_cali, raw, &mv) != ESP_OK) {
            return NAN;
        }
    } else {
        mv = (int)(((float)raw / 4095.0f) * TEMP_SENSOR_VREF * 1000.0f);
    }

    float v = (float)mv / 1000.0f;
    if (v <= 0.01f || v >= (TEMP_SENSOR_VREF - 0.01f)) {
        return NAN;
    }

#if TEMP_SENSOR_NTC_TO_VCC
    float r_ntc = TEMP_SENSOR_SERIES_RES_OHM * ((TEMP_SENSOR_VREF / v) - 1.0f);
#else
    float r_ntc = TEMP_SENSOR_SERIES_RES_OHM * (v / (TEMP_SENSOR_VREF - v));
#endif
    if (r_ntc <= 0.0f) {
        return NAN;
    }

    float inv_t = (1.0f / TEMP_SENSOR_T0_K) +
                  (logf(r_ntc / TEMP_SENSOR_NTC_R0_OHM) / TEMP_SENSOR_NTC_BETA);
    float temp_k = 1.0f / inv_t;
    float temp_c = temp_k - 273.15f;

    if (log_debug) {
        ESP_LOGI(TAG, "NTC dbg: raw=%d mv=%d v=%.3f r=%.0f t=%.2fC", raw, mv, v, r_ntc, temp_c);
    }
    return temp_c;
}

static float temperature_read_c(void) {
    return temperature_read_c_internal(true);
}

static float temperature_read_c_quiet(void) {
    return temperature_read_c_internal(false);
}

#endif
