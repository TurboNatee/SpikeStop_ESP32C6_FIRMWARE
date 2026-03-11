idf_component_register(
    SRCS "esp32wifiap.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
    esp_wifi
    wifi_provisioning
    bt
    esp_netif
    nvs_flash
    lwip
    driver
    esp_event
    freertos
    esp_timer
    led_strip
    esp_http_client
    mbedtls
    esp_adc
)
