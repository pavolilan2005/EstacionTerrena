#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @file lora_sx127x.h
 * @brief Interfaz del controlador receptor LoRa SX1276/SX1278.
 */

/** @brief Paquete LoRa recibido y sus metadatos de enlace. */
typedef struct {
    char payload[192];   /**< Carga útil terminada en NUL. */
    size_t length;       /**< Cantidad de bytes recibidos, sin contar el NUL. */
    int rssi_dbm;         /**< RSSI del paquete en dBm. */
    float snr_db;         /**< Relación señal/ruido del paquete en dB. */
    uint32_t packet_count;/**< Contador de paquetes recibidos desde el arranque. */
    uint32_t lost_count;  /**< Paquetes perdidos estimados por número de secuencia. */
    float loss_percent;   /**< Porcentaje acumulado de paquetes perdidos. */
    float cpu_percent;    /**< Uso combinado de CPU de los dos núcleos. */
    uint32_t error_count; /**< Tramas inválidas o fallos al enviar ACK. */
} lora_packet_t;

/**
 * @brief Configura el bus SPI y el SX127x, y comienza recepción continua.
 * @return ESP_OK al inicializar; un error de ESP-IDF si falla la configuración.
 */
esp_err_t lora_sx127x_init(void);

/**
 * @brief Lee un paquete disponible sin esperar bloqueado por una recepción.
 * @param[out] packet Estructura donde se almacenan la carga y metadatos.
 * @return true si se recibió un paquete; false si aún no hay uno disponible.
 */
bool lora_sx127x_receive(lora_packet_t *packet);

/** @brief Rearma el SX127x en recepción continua después de una falta de datos.
 * @return ESP_OK si el modo RX quedó activo; código de error en caso contrario.
 */
esp_err_t lora_sx127x_retry_receive(void);

/**
 * @brief Envía el acuse `$A,<secuencia>,OK` y vuelve a RX continuo.
 * @param[in] sequence Secuencia que se confirma al nodo transmisor.
 * @return ESP_OK al completar la transmisión; código ESP-IDF si falla.
 */
esp_err_t lora_sx127x_send_ack(uint8_t sequence);
