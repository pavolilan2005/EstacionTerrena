/** @file lora_sx127x.c @brief Acceso por SPI y recepción de paquetes SX127x. */
#include "lora_sx127x.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** @name Conexión física y bus del módulo LoRa
 * Pines definidos para el cableado del prototipo ESP32 DevKit V1.
 * @{ */
#define LORA_SPI_HOST       SPI2_HOST /**< Bus SPI2 compartido con la TFT. */
#define LORA_PIN_CS         5
#define LORA_PIN_RESET     14
#define LORA_PIN_DIO0       26
#define LORA_SPI_HZ   8000000 /**< Reloj SPI del SX127x en Hz. */
/** @} */

/** @name Registros SX1276/SX1278
 * Direcciones del mapa de registros usado por el controlador.
 * @{ */
#define REG_FIFO             0x00
#define REG_OP_MODE          0x01
#define REG_FRF_MSB          0x06
#define REG_FIFO_ADDR_PTR    0x0D
#define REG_FIFO_TX_BASE     0x0E
#define REG_FIFO_RX_BASE     0x0F
#define REG_FIFO_RX_CURRENT  0x10
#define REG_IRQ_FLAGS        0x12
#define REG_RX_NB_BYTES      0x13
#define REG_PKT_SNR_VALUE    0x19
#define REG_PKT_RSSI_VALUE   0x1A
#define REG_MODEM_CONFIG_1   0x1D
#define REG_MODEM_CONFIG_2   0x1E
#define REG_PREAMBLE_MSB     0x20
#define REG_PREAMBLE_LSB     0x21
#define REG_PAYLOAD_LENGTH   0x22
#define REG_MAX_PAYLOAD_LEN  0x23
#define REG_MODEM_CONFIG_3   0x26
#define REG_SYNC_WORD        0x39
#define REG_VERSION          0x42
#define REG_PA_CONFIG        0x09
#define REG_OCP              0x0B
#define REG_LNA              0x0C
/** @} */

/** @name Modos de operación
 * Bits de modo LoRa escritos en el registro REG_OP_MODE.
 * @{ */
#define MODE_LONG_RANGE      0x80
#define MODE_SLEEP           0x00
#define MODE_STANDBY         0x01
#define MODE_RX_CONTINUOUS   0x05
/** @} */

/** Etiqueta para mensajes del controlador SX127x. */
static const char *TAG = "sx127x";
/** Manejador SPI registrado para el transceptor. */
static spi_device_handle_t radio_spi;
/** Paquetes válidos recibidos desde el arranque. */
static uint32_t received_count;

/**
 * @brief Ejecuta una transferencia SPI de 16 bits a un registro SX127x.
 * @param[in] address Dirección del registro de 7 bits.
 * @param[in] value Byte enviado como dato durante escritura.
 * @param[in] write true para escribir; false para leer.
 * @param[out] result Destino opcional del byte leído; puede ser NULL.
 * @return Resultado de spi_device_polling_transmit.
 */
static esp_err_t transfer_reg(uint8_t address, uint8_t value, bool write,
                              uint8_t *result)
{
    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = 16;
    transaction.tx_data[0] = write ? (address | 0x80) : (address & 0x7F);
    transaction.tx_data[1] = value;

    esp_err_t err = spi_device_polling_transmit(radio_spi, &transaction);
    if (err == ESP_OK && result != NULL) {
        *result = transaction.rx_data[1];
    }
    return err;
}

/** @brief Escribe un byte en un registro del SX127x.
 * @param[in] address Dirección del registro.
 * @param[in] value Byte a escribir.
 * @return Resultado de la transferencia SPI.
 */
static esp_err_t write_reg(uint8_t address, uint8_t value)
{
    return transfer_reg(address, value, true, NULL);
}

/** @brief Lee un byte de un registro del SX127x.
 * @param[in] address Dirección del registro.
 * @param[out] value Byte leído.
 * @return Resultado de la transferencia SPI.
 */
static esp_err_t read_reg(uint8_t address, uint8_t *value)
{
    return transfer_reg(address, 0, false, value);
}

/**
 * @brief Lee bytes consecutivos de la FIFO de recepción.
 * @param[out] data Búfer de destino con espacio para `length` bytes.
 * @param[in] length Longitud solicitada; debe estar entre 1 y 255.
 * @return ESP_ERR_INVALID_ARG para una longitud fuera de rango; en otro caso,
 *         resultado de la transferencia SPI.
 */
static esp_err_t read_fifo(uint8_t *data, size_t length)
{
    if (length == 0 || length > 255) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[256] = {0};
    uint8_t rx[256] = {0};
    tx[0] = REG_FIFO & 0x7F;

    spi_transaction_t transaction = {0};
    transaction.length = (length + 1) * 8;
    transaction.rxlength = (length + 1) * 8;
    transaction.tx_buffer = tx;
    transaction.rx_buffer = rx;
    esp_err_t err = spi_device_polling_transmit(radio_spi, &transaction);
    if (err == ESP_OK) {
        memcpy(data, &rx[1], length);
    }
    return err;
}

/** @brief Cambia el SX127x a un modo LoRa indicado.
 * @param[in] mode Modo sleep, standby o recepción continua.
 * @return Resultado de escribir REG_OP_MODE.
 */
static esp_err_t set_mode(uint8_t mode)
{
    return write_reg(REG_OP_MODE, MODE_LONG_RANGE | mode);
}

/**
 * @brief Inicializa GPIO/SPI, programa el perfil de radio y habilita RX continuo.
 * @return ESP_OK al completar el arranque; error ESP-IDF ante cualquier fallo.
 *
 * Perfil actual: 433 MHz, BW 125 kHz, SF7, CR 4/5, CRC y sync word 0x12.
 */
esp_err_t lora_sx127x_init(void)
{
    spi_device_interface_config_t spi_config = {
        .clock_speed_hz = LORA_SPI_HZ,
        .mode = 0,
        .spics_io_num = LORA_PIN_CS,
        .queue_size = 1,
        .flags = 0,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(LORA_SPI_HOST, &spi_config, &radio_spi),
                        TAG, "No se pudo agregar SX1278 al bus SPI");

    ESP_RETURN_ON_ERROR(gpio_set_direction(LORA_PIN_RESET, GPIO_MODE_OUTPUT),
                        TAG, "No se pudo configurar RESET");
    ESP_RETURN_ON_ERROR(gpio_set_level(LORA_PIN_RESET, 0), TAG, "RESET bajo fallo");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(gpio_set_level(LORA_PIN_RESET, 1), TAG, "RESET alto fallo");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(gpio_set_direction(LORA_PIN_DIO0, GPIO_MODE_INPUT),
                        TAG, "No se pudo configurar DIO0");

    uint8_t version = 0;
    ESP_RETURN_ON_ERROR(read_reg(REG_VERSION, &version), TAG, "No se pudo leer VERSION");
    if (version != 0x12) {
        ESP_LOGE(TAG, "Version SX127x inesperada: 0x%02X (se esperaba 0x12)", version);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_RETURN_ON_ERROR(set_mode(MODE_SLEEP), TAG, "No se pudo poner en sleep");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(set_mode(MODE_STANDBY), TAG, "No se pudo poner en standby");

    // Perfil de prueba: 433 MHz, BW 125 kHz, SF7, CR 4/5, cabecera explicita.
    const uint64_t frf = ((uint64_t)433000000 * (1ULL << 19)) / 32000000;
    ESP_RETURN_ON_ERROR(write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF), TAG, "FRF MSB fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_FRF_MSB + 1, (frf >> 8) & 0xFF), TAG, "FRF MID fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_FRF_MSB + 2, frf & 0xFF), TAG, "FRF LSB fallo");

    ESP_RETURN_ON_ERROR(write_reg(REG_FIFO_TX_BASE, 0x00), TAG, "FIFO TX base fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_FIFO_RX_BASE, 0x00), TAG, "FIFO RX base fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_MODEM_CONFIG_1, 0x72), TAG, "ModemConfig1 fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_MODEM_CONFIG_2, 0x74), TAG, "ModemConfig2 fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_MODEM_CONFIG_3, 0x04), TAG, "ModemConfig3 fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_PREAMBLE_MSB, 0x00), TAG, "Preamble MSB fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_PREAMBLE_LSB, 0x08), TAG, "Preamble LSB fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_MAX_PAYLOAD_LEN, 0xFF), TAG, "Max payload fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_SYNC_WORD, 0x12), TAG, "Sync word fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_LNA, 0x23), TAG, "LNA fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_PA_CONFIG, 0x8F), TAG, "PA config fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_OCP, 0x2B), TAG, "OCP fallo");
    ESP_RETURN_ON_ERROR(write_reg(REG_IRQ_FLAGS, 0xFF), TAG, "No se pudieron limpiar IRQ");
    ESP_RETURN_ON_ERROR(set_mode(MODE_RX_CONTINUOUS), TAG, "No se pudo iniciar RX continuo");

    ESP_LOGI(TAG, "SX1278 listo: 433 MHz / SF7 / BW125 / CR4/5 / CRC / sync 0x12");
    return ESP_OK;
}

/**
 * @brief Comprueba DIO0 y copia un paquete LoRa disponible desde la FIFO.
 * @return true para un paquete válido; false si no hay RxDone o hubo error CRC.
 *
 * Esta rutina es de sondeo: no espera a que llegue un paquete, para permitir
 * que la tarea de radio atienda periódicamente otras tareas del sistema.
 */
bool lora_sx127x_receive(lora_packet_t *packet)
{
    // DIO0 se asigna a RxDone con el mapeo LoRa por defecto; lo sondeamos desde la tarea.
    if (gpio_get_level(LORA_PIN_DIO0) == 0) {
        return false;
    }
    uint8_t irq_flags = 0;
    if (read_reg(REG_IRQ_FLAGS, &irq_flags) != ESP_OK || !(irq_flags & 0x40)) {
        return false;
    }

    // RxDone se limpia siempre; descartamos paquetes con error de CRC.
    (void)write_reg(REG_IRQ_FLAGS, irq_flags);
    if (irq_flags & 0x20) {
        ESP_LOGW(TAG, "Paquete recibido con error CRC");
        return false;
    }

    uint8_t length = 0;
    uint8_t fifo_address = 0;
    if (read_reg(REG_RX_NB_BYTES, &length) != ESP_OK ||
        read_reg(REG_FIFO_RX_CURRENT, &fifo_address) != ESP_OK || length == 0) {
        return false;
    }

    if (write_reg(REG_FIFO_ADDR_PTR, fifo_address) != ESP_OK ||
        read_fifo((uint8_t *)packet->payload, length) != ESP_OK) {
        return false;
    }

    packet->length = length < sizeof(packet->payload) - 1
                         ? length
                         : sizeof(packet->payload) - 1;
    packet->payload[packet->length] = '\0';

    uint8_t raw_rssi = 0;
    uint8_t raw_snr = 0;
    (void)read_reg(REG_PKT_RSSI_VALUE, &raw_rssi);
    (void)read_reg(REG_PKT_SNR_VALUE, &raw_snr);
    packet->rssi_dbm = (int)raw_rssi - 164;
    packet->snr_db = ((int8_t)raw_snr) / 4.0f;
    packet->packet_count = ++received_count;
    return true;
}

/**
 * @brief Limpia interrupciones y vuelve a poner el transceptor en RX continuo.
 * @return ESP_OK si se reanudó la escucha; error ESP-IDF si falla una escritura.
 */
esp_err_t lora_sx127x_retry_receive(void)
{
    ESP_RETURN_ON_ERROR(set_mode(MODE_STANDBY), TAG, "No se pudo rearmar SX1278");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(write_reg(REG_IRQ_FLAGS, 0xFF), TAG,
                        "No se pudieron limpiar IRQ al rearmar");
    ESP_RETURN_ON_ERROR(set_mode(MODE_RX_CONTINUOUS), TAG,
                        "No se pudo volver a RX continuo");
    ESP_LOGW(TAG, "Sin paquetes: receptor rearmado, sigue escuchando");
    return ESP_OK;
}
