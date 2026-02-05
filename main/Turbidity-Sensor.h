#ifndef TURBIDITY_SENSOR_H
#define TURBIDITY_SENSOR_H

#include "esp_adc/adc_oneshot.h"

// Turbidity Sensor (ADC)
// ESP32-C6: ADC_UNIT_1, ADC_CHANNEL_0 = GPIO1
static bool turbidity_sensor_present = false;
#define TURBIDITY_CH ADC_CHANNEL_0
#define CLEAR_THRESHOLD   2.90f
#define CLOUDY_THRESHOLD  1.50f
static adc_oneshot_unit_handle_t adc_handle;

static void turbidity_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t ch_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, TURBIDITY_CH, &ch_cfg));
    ESP_LOGI(TAG, "Turbidity ADC init @ GPIO1");
}

static float turbidity_read_voltage(void) {
    int raw = 0;
    adc_oneshot_read(adc_handle, TURBIDITY_CH, &raw);
    return (raw / 4095.0f) * 3.3f;
}

static int turbidity_get_status(float v) {
    if (v > CLEAR_THRESHOLD) return 0;
    else if (v > CLOUDY_THRESHOLD) return 1;
    else return 2;
}

#endif // TURBIDITY_SENSOR_H
