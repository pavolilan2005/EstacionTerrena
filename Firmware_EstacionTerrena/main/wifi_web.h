#pragma once

#include "esp_err.h"
#include "lora_sx127x.h"

/**
 * @file wifi_web.h
 * @brief Punto de acceso Wi-Fi y panel local de telemetría.
 */

/** @brief Inicia el punto de acceso y el servidor HTTP del panel web.
 * @return ESP_OK si Wi-Fi y el servidor HTTP quedaron activos.
 */
esp_err_t wifi_web_init(void);

/** @brief Publica la lectura más reciente para el panel web.
 * @param[in] packet Paquete recién recibido; se copia para acceso concurrente.
 */
void wifi_web_update(const lora_packet_t *packet);
