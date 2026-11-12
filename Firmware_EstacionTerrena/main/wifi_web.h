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

/**
 * @brief Actualiza métricas globales del panel HTTP sin esperar otro paquete.
 * @param[in] loss_percent Pérdida acumulada en porcentaje.
 * @param[in] cpu_percent Uso combinado de CPU en porcentaje.
 * @param[in] errors Errores de trama y transmisiones ACK fallidas.
 */
void wifi_web_update_metrics(float loss_percent, float cpu_percent, uint32_t errors);
