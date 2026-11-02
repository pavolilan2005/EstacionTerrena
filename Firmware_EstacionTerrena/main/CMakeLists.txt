# Fuentes del receptor, TFT, controlador LoRa y panel HTTP/Wi-Fi.
idf_component_register(
    SRCS "wifi_web.c" "main.c" "display.c" "lora_sx127x.c"
    INCLUDE_DIRS "."
    # Controladores ESP-IDF y servicios requeridos por cada módulo.
    REQUIRES esp_driver_gpio esp_driver_spi esp_lcd esp_event esp_netif esp_wifi esp_http_server nvs_flash
)
