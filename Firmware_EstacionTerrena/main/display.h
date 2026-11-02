#pragma once

#include "lora_sx127x.h"
#include "esp_err.h"

/**
 * @file display.h
 * @brief Interfaz de la pantalla TFT ST7789 y su panel de telemetría.
 */

/** @brief Inicializa el controlador ST7789 en el bus SPI compartido.
 * @return ESP_OK si se pudo inicializar la pantalla.
 */
esp_err_t display_init(void);

/** @brief Dibuja la interfaz inicial y el estado de espera de paquetes. */
void display_show_waiting(void);

/** @brief Actualiza los valores y el historial a partir del paquete recibido.
 * @param[in] packet Paquete LoRa con telemetría y métricas de enlace.
 */
void display_show_packet(const lora_packet_t *packet);

/** @brief Actualiza el indicador cuando vence el tiempo sin recibir paquetes. */
void display_check_link_timeout(void);
