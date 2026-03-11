#ifndef TURBIDITY_SENSOR_H
#define TURBIDITY_SENSOR_H

#include "esp_adc/adc_oneshot.h"

static bool turbidity_sensor_present = false;
#ifndef TURBIDITY_SENSOR_GPIO
#define TURBIDITY_SENSOR_GPIO 0
#endif
#define CLEAR_THRESHOLD   2.90f
#define CLOUDY_THRESHOLD  1.50f
static adc_oneshot_unit_handle_t adc_handle;
static bool adc1_unit_ready = false;
static adc_channel_t turbidity_adc_channel = ADC_CHANNEL_0;

static adc_oneshot_unit_handle_t adc_get_or_init_unit1(void) {
    if (!adc1_unit_ready) {
        adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
        if (adc_oneshot_new_unit(&init_cfg, &adc_handle) != ESP_OK) {
            return NULL;
        }
        adc1_unit_ready = true;
    }
    return adc_handle;
}

static void turbidity_init(void) {
    adc_unit_t unit = ADC_UNIT_1;
    if (adc_oneshot_io_to_channel(TURBIDITY_SENSOR_GPIO, &unit, &turbidity_adc_channel) != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "Turbidity GPIO%d is not ADC1-capable", TURBIDITY_SENSOR_GPIO);
        turbidity_sensor_present = false;
        return;
    }

    if (adc_get_or_init_unit1() == NULL) {
        ESP_LOGE(TAG, "Failed to initialize ADC1 for turbidity");
        turbidity_sensor_present = false;
        return;
    }
    adc_oneshot_chan_cfg_t ch_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, turbidity_adc_channel, &ch_cfg));
    ESP_LOGI(TAG, "Turbidity ADC init @ GPIO%d", TURBIDITY_SENSOR_GPIO);
}

static float turbidity_read_voltage(void) {
    int raw = 0;
    adc_oneshot_read(adc_handle, turbidity_adc_channel, &raw);
    return (raw / 4095.0f) * 3.3f;
}

static int turbidity_get_status(float v) {
    if (v > CLEAR_THRESHOLD) {
        return 0;
    }
    if (v > CLOUDY_THRESHOLD) {
        return 1;
    }
    return 2;
}

#endif 
