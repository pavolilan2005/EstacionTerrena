/**
 * @file main.c
 * @brief Orquesta las tareas del receptor de la estación terrena.
 * @version 1.0.5
 *
 * Regla de versión: incrementa el PATCH de este encabezado en cada cambio de
 * firmware y conserva el mismo número en CMakeLists.txt y la documentación.
 * El sistema recibe telemetría LoRa, confirma cada secuencia, calcula métricas
 * y actualiza la TFT y el panel web mediante tareas FreeRTOS.
 */
#include "display.h"
#include "lora_sx127x.h"
#include "wifi_web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/* GPIO de indicadores; GPIO14 está reservado para RESET del SX1278. */
#define LED_MISSING GPIO_NUM_21
#define LED_OK      GPIO_NUM_22
#define LED_ALERT   GPIO_NUM_25
#define LINK_TIMEOUT_MS 5000

static const char *TAG = "ESTACION";       /**< Etiqueta común de logs de la aplicación. */
static QueueHandle_t packet_queue;           /**< Cola de una muestra para la pantalla. */
static uint32_t lost_total;                  /**< Pérdidas inferidas del contador de secuencia. */
static uint32_t expected_sequence;            /**< Próxima secuencia esperada, de 8 bits. */
static bool have_sequence;                    /**< Indica si ya se recibió la secuencia inicial. */
static uint32_t frame_errors;                 /**< Errores de trama y fallos de ACK acumulados. */
static uint32_t valid_packets;                /**< Tramas válidas aceptadas desde el arranque. */
static TickType_t last_valid_tick;             /**< Tick de la última trama LoRa válida. */
static volatile float cpu_percent;             /**< Uso estimado combinado de los dos núcleos. */
static volatile float loss_percent;            /**< Pérdida porcentual acumulada. */
static volatile uint32_t idle_ticks[2];        /**< Iteraciones de idle observadas por núcleo. */

/** @brief Cuenta iteraciones de CPU0 en el idle task para estimar carga. */
static bool idle_hook_cpu0(void)
{
    ++idle_ticks[0];
    return true;
}

/** @brief Cuenta iteraciones de CPU1 en el idle task para estimar carga. */
static bool idle_hook_cpu1(void)
{
    ++idle_ticks[1];
    return true;
}

/**
 * @brief Valida el checksum XOR de una trama ASCII.
 * @param[in] s Trama que comienza con `$` y contiene `*XX` al final.
 * @param[out] expected Checksum recibido, si la trama es válida.
 * @return true si el formato y el XOR son válidos.
 */
static bool checksum_ok(const char *s, uint8_t *expected)
{
    if (s == NULL || s[0] != '$') return false;
    const char *star = strchr(s, '*');
    if (star == NULL || strlen(star) < 3) return false;
    char *end = NULL;
    unsigned long received = strtoul(star + 1, &end, 16);
    if (end == star + 1 || received > 0xFF) return false;
    uint8_t sum = 0;
    for (const char *p = s + 1; p < star; ++p) sum ^= (uint8_t)*p;
    if (sum != (uint8_t)received) return false;
    *expected = (uint8_t)received;
    return true;
}

/**
 * @brief Verifica y separa los campos de la trama de sensores del nodo 2.
 * @param[in] frame Trama `$D,...*XX` terminada en NUL.
 * @param[out] seq Secuencia de 8 bits.
 * @param[out] v Once enteros de telemetría en las unidades transmitidas.
 * @return true si checksum, tipo, secuencia y todos los campos son válidos.
 *
 * Orden de campos: ax/ay/az en mg; gx/gy/gz en mdps; temperatura MPU en
 * centi-°C; presión en Pa; temperatura BME en centi-°C; humedad en milésimas
 * de porcentaje y resistencia de gas en ohmios.
 */
static bool parse_sensor_frame(const char *frame, uint8_t *seq, int32_t v[11])
{
    uint8_t check;
    if (!checksum_ok(frame, &check)) return false;
    char copy[192];
    size_t n = strcspn(frame, "*");
    if (n >= sizeof(copy)) return false;
    memcpy(copy, frame, n);
    copy[n] = '\0';
    char *save = NULL;
    char *token = strtok_r(copy + 1, ",", &save);
    if (token == NULL || strcmp(token, "D") != 0) return false;
    token = strtok_r(NULL, ",", &save);
    if (token == NULL) return false;
    char *end = NULL;
    long sequence = strtol(token, &end, 10);
    if (*end != '\0' || sequence < 0 || sequence > 255) return false;
    *seq = (uint8_t)sequence;
    for (int i = 0; i < 11; ++i) {
        token = strtok_r(NULL, ",", &save);
        if (token == NULL) return false;
        long value = strtol(token, &end, 10);
        if (*end != '\0' || value < INT32_MIN || value > INT32_MAX) return false;
        v[i] = (int32_t)value;
    }
    return strtok_r(NULL, ",", &save) == NULL;
}

/** @brief Añade una magnitud con divisor decimal al payload legible de la UI. */
static void append_scaled(char *dst, size_t cap, size_t *used, const char *key,
                          int32_t value, int32_t divisor, unsigned decimals)
{
    int32_t whole = value / divisor;
    int32_t fraction = abs(value % divisor);
    unsigned shown = decimals == 1 ? (unsigned)(fraction / (divisor / 10)) :
                    decimals == 2 ? (unsigned)(fraction / (divisor / 100)) :
                    (unsigned)fraction;
    int n = snprintf(dst + *used, cap - *used, "%s:%ld.%0*u ", key,
                     (long)whole, (int)decimals, shown);
    if (n > 0 && (size_t)n < cap - *used) *used += (size_t)n;
}

/** @brief Convierte la trama binaria de sensores a claves y unidades para la UI. */
static bool make_display_payload(lora_packet_t *p, uint8_t seq, const int32_t v[11])
{
    size_t used = 0;
    int n = snprintf(p->payload, sizeof(p->payload), "SEQ:%u ", (unsigned)seq);
    if (n < 0 || (size_t)n >= sizeof(p->payload)) return false;
    used = (size_t)n;
    append_scaled(p->payload, sizeof(p->payload), &used, "AX", v[0], 1000, 3);
    append_scaled(p->payload, sizeof(p->payload), &used, "AY", v[1], 1000, 3);
    append_scaled(p->payload, sizeof(p->payload), &used, "AZ", v[2], 1000, 3);
    append_scaled(p->payload, sizeof(p->payload), &used, "GX", v[3], 1000, 3);
    append_scaled(p->payload, sizeof(p->payload), &used, "GY", v[4], 1000, 3);
    append_scaled(p->payload, sizeof(p->payload), &used, "GZ", v[5], 1000, 3);
    append_scaled(p->payload, sizeof(p->payload), &used, "TM", v[6], 100, 2);
    append_scaled(p->payload, sizeof(p->payload), &used, "PRES", v[7], 100, 2);
    append_scaled(p->payload, sizeof(p->payload), &used, "TBME", v[8], 100, 2);
    append_scaled(p->payload, sizeof(p->payload), &used, "HUM", v[9], 1000, 3);
    int wrote = snprintf(p->payload + used, sizeof(p->payload) - used, "GAS:%ld", (long)v[10]);
    if (wrote < 0 || (size_t)wrote >= sizeof(p->payload) - used) return false;
    p->length = strlen(p->payload);
    return true;
}

/** @brief Actualiza pérdidas acumuladas considerando el wrap de secuencia de 8 bits. */
static void update_loss(uint8_t sequence)
{
    if (have_sequence) {
        uint8_t gap = (uint8_t)(sequence - (uint8_t)expected_sequence);
        if (gap < 128) lost_total += gap;
    }
    expected_sequence = (uint8_t)(sequence + 1);
    have_sequence = true;
}

/**
 * @brief Tarea FreeRTOS de recepción LoRa, validación, ACK y publicación.
 * @param[in] arg No utilizado.
 *
 * La recepción se sondea sin bloqueos largos; una trama válida se confirma
 * con `$A,<seq>,OK`, actualiza LEDs y se envía a la cola de pantalla y al panel.
 */
static void radio_task(void *arg)
{
    (void)arg;
    lora_packet_t packet = {0};
    last_valid_tick = xTaskGetTickCount();
    for (;;) {
        if (lora_sx127x_receive(&packet)) {
            uint8_t sequence = 0;
            int32_t values[11];
            if (!parse_sensor_frame(packet.payload, &sequence, values)) {
                ++frame_errors;
                ESP_LOGW(TAG, "Trama LoRa invalida/CRC XOR; descartada");
                (void)gpio_set_level(LED_ALERT, 1);
                vTaskDelay(pdMS_TO_TICKS(40));
                (void)gpio_set_level(LED_ALERT, 0);
                continue;
            }
            update_loss(sequence);
            ++valid_packets;
            packet.lost_count = lost_total;
            packet.loss_percent = (valid_packets + lost_total) ?
                100.0f * (float)lost_total / (float)(valid_packets + lost_total) : 0.0f;
            packet.cpu_percent = cpu_percent;
            packet.error_count = frame_errors;
            if (!make_display_payload(&packet, sequence, values)) {
                ++frame_errors;
                continue;
            }
            last_valid_tick = xTaskGetTickCount();
            (void)gpio_set_level(LED_MISSING, 0);
            (void)gpio_set_level(LED_OK, 1);
            esp_err_t ack_err = lora_sx127x_send_ack(sequence);
            if (ack_err != ESP_OK) {
                ++frame_errors;
                (void)gpio_set_level(LED_ALERT, 1);
                ESP_LOGW(TAG, "No se pudo enviar ACK seq=%u: %s", sequence, esp_err_to_name(ack_err));
            } else {
                ESP_LOGI(TAG, "RX seq=%u RSSI=%d SNR=%.1f dB; ACK enviado",
                         sequence, packet.rssi_dbm, (double)packet.snr_db);
                (void)gpio_set_level(LED_ALERT, 0);
            }
            wifi_web_update(&packet);
            (void)xQueueOverwrite(packet_queue, &packet);
        } else {
            TickType_t age = xTaskGetTickCount() - last_valid_tick;
            if (age >= pdMS_TO_TICKS(LINK_TIMEOUT_MS)) {
                (void)gpio_set_level(LED_OK, 0);
                (void)gpio_set_level(LED_MISSING, ((xTaskGetTickCount() / pdMS_TO_TICKS(250)) & 1U));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/**
 * @brief Calcula cada segundo el uso combinado de CPU y pérdida acumulada.
 * @param[in] arg No utilizado.
 *
 * Usa contadores de los hooks idle de ambos núcleos y no necesita activar
 * CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS.
 */
static void cpu_task(void *arg)
{
    (void)arg;
    uint32_t previous_idle[2] = {idle_ticks[0], idle_ticks[1]};
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now_idle[2] = {idle_ticks[0], idle_ticks[1]};
        uint32_t idle_delta = (now_idle[0] - previous_idle[0]) +
                              (now_idle[1] - previous_idle[1]);
        const uint32_t possible_idle = 2U * (uint32_t)configTICK_RATE_HZ;
        if (possible_idle != 0) {
            if (idle_delta > possible_idle) idle_delta = possible_idle;
            cpu_percent = 100.0f * (float)(possible_idle - idle_delta) /
                          (float)possible_idle;
        }
        previous_idle[0] = now_idle[0];
        previous_idle[1] = now_idle[1];
        loss_percent = (valid_packets + lost_total) ?
                       100.0f * (float)lost_total / (float)(valid_packets + lost_total) : 0.0f;
        wifi_web_update_metrics(loss_percent, cpu_percent, frame_errors);
    }
}

/** @brief Consume la última muestra de la cola y refresca estado y métricas TFT. */
static void display_task(void *arg)
{
    (void)arg;
    lora_packet_t packet;
    for (;;) {
        if (xQueueReceive(packet_queue, &packet, pdMS_TO_TICKS(250)) == pdTRUE) {
            display_show_packet(&packet);
        }
        display_update_metrics(loss_percent, cpu_percent);
        display_check_link_timeout();
    }
}

/** @brief Configura como salidas los tres LED de estado e inicia su estado seguro. */
static void gpio_init_status(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_MISSING) | (1ULL << LED_OK) | (1ULL << LED_ALERT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_set_level(LED_MISSING, 1));
    ESP_ERROR_CHECK(gpio_set_level(LED_OK, 0));
    ESP_ERROR_CHECK(gpio_set_level(LED_ALERT, 0));
}

/**
 * @brief Punto de entrada de ESP-IDF: inicializa periféricos y tareas.
 *
 * El orden reserva primero el bus/pantalla y radio, después levanta el AP web,
 * crea una cola de una muestra y finalmente inicia las tareas de aplicación.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Estacion terrena iniciando; firmware v1.0.5");
    ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_hook_cpu0, 0));
    ESP_ERROR_CHECK(esp_register_freertos_idle_hook_for_cpu(idle_hook_cpu1, 1));
    gpio_init_status();
    ESP_ERROR_CHECK(display_init());
    display_show_waiting();
    ESP_ERROR_CHECK(lora_sx127x_init());
    ESP_ERROR_CHECK(wifi_web_init());
    packet_queue = xQueueCreate(1, sizeof(lora_packet_t));
    configASSERT(packet_queue != NULL);
    configASSERT(xTaskCreatePinnedToCore(radio_task, "lora_rx", 6144, NULL, 8, NULL, 1) == pdPASS);
    configASSERT(xTaskCreatePinnedToCore(display_task, "tft_ui", 6144, NULL, 4, NULL, 1) == pdPASS);
    configASSERT(xTaskCreatePinnedToCore(cpu_task, "cpu_stats", 3072, NULL, 2, NULL, 0) == pdPASS);
    ESP_LOGI(TAG, "AP ASTRA-I-RX listo: http://192.168.4.1");
}
