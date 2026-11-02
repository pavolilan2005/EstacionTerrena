/**
 * @file main.c
 * @brief Arranque del receptor y coordinación de las tareas FreeRTOS.
 *
 * @mainpage ASTRA I - Estación receptora LoRa
 * El ESP32 recibe paquetes SX1278, mantiene el último dato en el panel web
 * Wi-Fi y envía el paquete más reciente a la pantalla ST7789. La recepción de
 * radio corre en una tarea prioritaria; la interfaz gráfica se actualiza en
 * una tarea separada mediante un buzón FreeRTOS de un elemento, evitando que
 * el tiempo de dibujo detenga la recepción.
 *
 * @section hw Hardware y pines
 * SPI compartido: SCK GPIO18, MISO GPIO19, MOSI GPIO23. SX1278: CS GPIO5,
 * RESET GPIO14, DIO0 GPIO26. ST7789: CS GPIO15, DC GPIO2, RESET GPIO4.
 * Ambos módulos se alimentan a 3.3 V y comparten GND.
 *
 * @section payload Formato de telemetría
 * Se recomienda una carga ASCII con campos separados por espacios, por
 * ejemplo `SEQ:1234 TBME:24.6 PRES:1012 HUM:48 GAS:120000 AX:0.01 AY:0.02
 * AZ:0.99 GX:-2.8 GY:-1.0 GZ:0.5 TM:26.7`. Los nombres aceptados y unidades
 * se describen en el código de la interfaz. `SEQ` permite estimar pérdidas.
 *
 * @section wifi Acceso Wi-Fi local
 * El receptor crea el AP `ASTRA-I-RX`; el panel se abre en http://192.168.4.1.
 * SSID y contraseña configurables se encuentran en wifi_web.c.
 */

#include "display.h"
#include "lora_sx127x.h"
#include "wifi_web.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/** Etiqueta común de los mensajes de registro de la aplicación. */
static const char *TAG = "receptor";
/** Buzón FreeRTOS de un elemento que conserva siempre el paquete más reciente. */
static QueueHandle_t packet_queue;
/** Siguiente número de secuencia esperado en el flujo LoRa. */
static uint32_t next_sequence;
/** Acumulado de paquetes perdidos inferido a partir de los campos SEQ. */
static uint32_t total_lost;
/** Indica si ya se recibió una secuencia inicial para poder calcular huecos. */
static bool sequence_initialized;

/**
 * @brief Busca el campo SEQ en la carga y extrae su número decimal.
 * @param[in] payload Carga ASCII del paquete, con campos separados por delimitadores.
 * @param[out] sequence Número encontrado si la función devuelve true.
 * @return true si encontró `SEQ:` o `SEQ=` seguido de un número.
 */
static bool extract_sequence(const char *payload, uint32_t *sequence)
{
    for (const char *cursor = payload; *cursor; ++cursor) {
        if (cursor[0] != '\0' && cursor[1] != '\0' && cursor[2] != '\0' &&
            cursor[3] != '\0' && toupper((unsigned char)cursor[0]) == 'S' &&
            toupper((unsigned char)cursor[1]) == 'E' &&
            toupper((unsigned char)cursor[2]) == 'Q' &&
            (cursor[3] == ':' || cursor[3] == '=')) {
            char *end = NULL;
            unsigned long value = strtoul(cursor + 4, &end, 10);
            if (end != cursor + 4) {
                *sequence = (uint32_t)value;
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Tarea de mayor prioridad: sondea LoRa, calcula pérdidas y publica datos.
 * @param[in] argument No utilizado; requerido por la firma de FreeRTOS.
 *
 * La tarea se mantiene no bloqueante respecto de la pantalla mediante
 * xQueueOverwrite y rearma el receptor cada cinco segundos si no hay paquetes.
 */
static void radio_task(void *argument)
{
    (void)argument;
    lora_packet_t packet;
    TickType_t last_rx_tick = xTaskGetTickCount();
    TickType_t next_retry_tick = last_rx_tick + pdMS_TO_TICKS(5000);

    while (true) {
        if (lora_sx127x_receive(&packet)) {
            last_rx_tick = xTaskGetTickCount();
            next_retry_tick = last_rx_tick + pdMS_TO_TICKS(5000);
            uint32_t sequence = 0;
            if (extract_sequence(packet.payload, &sequence)) {
                if (sequence_initialized) {
                    uint32_t gap = sequence - next_sequence;
                    if (gap < 0x80000000U) total_lost += gap;
                }
                next_sequence = sequence + 1;
                sequence_initialized = true;
            }
            packet.lost_count = total_lost;
            wifi_web_update(&packet);
            int snr_tenths = (int)(packet.snr_db * 10.0f);
            ESP_LOGI(TAG, "RX #%lu, lost=%lu, %u bytes, RSSI %d dBm, SNR %d.%d dB",
                     (unsigned long)packet.packet_count,
                     (unsigned long)packet.lost_count,
                     (unsigned)packet.length,
                     packet.rssi_dbm,
                     snr_tenths / 10,
                     snr_tenths < 0 ? -(snr_tenths % 10) : snr_tenths % 10);
            ESP_LOG_BUFFER_HEXDUMP(TAG, packet.payload, packet.length, ESP_LOG_INFO);
            // La pantalla es mas lenta que LoRa: conserva solo la lectura mas reciente.
            // Asi la tarea de radio nunca espera a que termine el dibujo.
            xQueueOverwrite(packet_queue, &packet);
        } else {
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(now - next_retry_tick) >= 0) {
                esp_err_t err = lora_sx127x_retry_receive();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Reintento de RX fallo: %s", esp_err_to_name(err));
                }
                last_rx_tick = now;
                next_retry_tick = now + pdMS_TO_TICKS(5000);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Tarea de interfaz que consume el último paquete y comprueba timeout.
 * @param[in] argument No utilizado; requerido por la firma de FreeRTOS.
 */
static void display_task(void *argument)
{
    (void)argument;
    lora_packet_t packet;
    while (true) {
        if (xQueueReceive(packet_queue, &packet, pdMS_TO_TICKS(250)) == pdTRUE) {
            display_show_packet(&packet);
        }
        display_check_link_timeout();
    }
}

/**
 * @brief Punto de entrada de ESP-IDF; inicializa periféricos y tareas.
 *
 * La cola se crea antes de lanzar las tareas para que la tarea de radio pueda
 * publicar paquetes de forma segura desde su primera iteración.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando receptor LoRa + TFT");
    ESP_ERROR_CHECK(display_init());
    display_show_waiting();
    ESP_ERROR_CHECK(lora_sx127x_init());
    ESP_ERROR_CHECK(wifi_web_init());

    // Buzon de una posicion: si llegan varios paquetes mientras se dibuja,
    // la pantalla recibe el mas reciente al terminar.
    packet_queue = xQueueCreate(1, sizeof(lora_packet_t));
    if (packet_queue == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    configASSERT(xTaskCreate(radio_task, "lora_rx", 4096, NULL, 5, NULL) == pdPASS);
    configASSERT(xTaskCreate(display_task, "tft_ui", 4096, NULL, 4, NULL) == pdPASS);
    ESP_LOGI(TAG, "Tareas FreeRTOS iniciadas");
}
