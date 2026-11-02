/** @file display.c @brief Dibujo del panel vertical y gráfica de temperatura. */
#include "display.h"

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_vendor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/** @name Pines, resolución y geometría de la interfaz TFT
 * Valores correspondientes al cableado y al panel ST7789 de 240 x 320.
 * @{ */
#define PIN_SPI_SCK       18
#define PIN_SPI_MOSI      23
#define PIN_SPI_MISO      19
#define PIN_TFT_CS        15
#define PIN_TFT_DC         2
#define PIN_TFT_RST        4
#define TFT_WIDTH        240
#define TFT_HEIGHT       320
#define DRAW_CHUNK        16
#define GRAPH_WIDTH      220
#define GRAPH_HEIGHT      56
#define GRAPH_X           10
#define GRAPH_Y          178
/** @} */

/** @name Paleta RGB565 de la interfaz
 * @{ */
#define COLOR_BG       0x0000
#define COLOR_WHITE    0xFFFF
#define COLOR_CYAN     0x07FF
#define COLOR_GREEN    0x07E0
#define COLOR_YELLOW   0xFFE0
#define COLOR_GRAY     0x9CF3
#define COLOR_CARD     0x07FF
#define COLOR_DATA     0x0010
#define COLOR_PLOT     0x0861
#define COLOR_BLUE     0x051F
/** @} */

/** Etiqueta para los mensajes del subsistema de pantalla. */
static const char *TAG = "display";
/** Manejador del panel ST7789 inicializado por ESP-IDF. */
static esp_lcd_panel_handle_t panel;
/** Búfer reutilizable para transferir bloques de color a la TFT. */
static uint16_t fill_chunk[TFT_WIDTH * DRAW_CHUNK];
/** Búfer de píxeles de un carácter de tamaño normal. */
static uint16_t char_pixels[6 * 8];
/** Búfer de píxeles para caracteres ampliados hasta escala 2x. */
static uint16_t large_char_pixels[10 * 14];
/** Semáforo binario dado por el callback al terminar la transferencia DMA. */
static SemaphoreHandle_t color_done_sem;

/**
 * @brief Callback ISR notificado al finalizar una transferencia de color.
 * @param[in] panel_io Interfaz de E/S de pantalla; no se necesita en el callback.
 * @param[in] event_data Datos del evento; no se necesitan para sincronizar.
 * @param[in] user_ctx Semáforo binario pasado al crear la interfaz LCD.
 * @return true si FreeRTOS debe solicitar un cambio de contexto al salir de ISR.
 */
static bool color_transfer_done(esp_lcd_panel_io_handle_t panel_io,
                                esp_lcd_panel_io_event_data_t *event_data,
                                void *user_ctx)
{
    (void)panel_io;
    (void)event_data;
    SemaphoreHandle_t done = (SemaphoreHandle_t)user_ctx;
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(done, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

/**
 * @brief Envía un rectángulo de píxeles y espera la finalización de su DMA.
 * @param[in] x1 Coordenada horizontal inicial, incluida.
 * @param[in] y1 Coordenada vertical inicial, incluida.
 * @param[in] x2 Coordenada horizontal final, exclusiva.
 * @param[in] y2 Coordenada vertical final, exclusiva.
 * @param[in] pixels Búfer RGB565 con los píxeles del rectángulo.
 */
static void draw_bitmap_sync(int x1, int y1, int x2, int y2, const void *pixels)
{
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, x1, y1, x2, y2, pixels));
    configASSERT(xSemaphoreTake(color_done_sem, pdMS_TO_TICKS(1000)) == pdTRUE);
}
/** Búfer RGB565 con el área de la gráfica de temperatura. */
static uint16_t graph_pixels[GRAPH_WIDTH * GRAPH_HEIGHT];
/** Última coordenada vertical de temperatura dibujada. */
static int graph_last_y;
/** Número de muestras históricas ya representadas en la gráfica. */
static unsigned graph_sample_count;
/** Estado del enlace LoRa reflejado en el pie de pantalla. */
static bool link_online;
/** Tick FreeRTOS en que se recibió el último paquete mostrado. */
static TickType_t last_packet_tick;
/** Número acumulado de paquetes recibidos que muestra la pantalla. */
static uint32_t displayed_received;
/** Número de paquetes perdidos estimados que muestra la pantalla. */
static uint32_t displayed_lost;
/** RSSI del último paquete mostrado, en dBm. */
static int displayed_rssi;
/** Índice de la página activa: 0 ambiente, 1 movimiento. */
static unsigned active_page;
/** Tick en que se dibujó la página actual, para alternar páginas. */
static TickType_t page_started_tick;

/** @brief Lecturas decodificadas para las dos páginas de telemetría. */
typedef struct {
    float bme_temp;  /**< Temperatura BME680 en grados Celsius. */
    float pressure;  /**< Presión atmosférica en hPa. */
    float humidity;  /**< Humedad relativa en porcentaje. */
    float gas;        /**< Resistencia de gas en ohmios. */
    float accel_x;    /**< Aceleración eje X, normalmente en g. */
    float accel_y;    /**< Aceleración eje Y, normalmente en g. */
    float accel_z;    /**< Aceleración eje Z, normalmente en g. */
    float gyro_x;     /**< Velocidad angular eje X, normalmente en DPS. */
    float gyro_y;     /**< Velocidad angular eje Y, normalmente en DPS. */
    float gyro_z;     /**< Velocidad angular eje Z, normalmente en DPS. */
    float mpu_temp;   /**< Temperatura interna del MPU6050 en Celsius. */
    bool has_bme_temp; /**< true si el paquete incluyó temperatura BME680. */
    bool has_pressure; /**< true si el paquete incluyó presión. */
    bool has_humidity; /**< true si el paquete incluyó humedad. */
    bool has_gas;      /**< true si el paquete incluyó resistencia de gas. */
    bool has_accel_x;  /**< true si el paquete incluyó aceleración X. */
    bool has_accel_y;  /**< true si el paquete incluyó aceleración Y. */
    bool has_accel_z;  /**< true si el paquete incluyó aceleración Z. */
    bool has_gyro_x;   /**< true si el paquete incluyó giroscopio X. */
    bool has_gyro_y;   /**< true si el paquete incluyó giroscopio Y. */
    bool has_gyro_z;   /**< true si el paquete incluyó giroscopio Z. */
    bool has_mpu_temp; /**< true si el paquete incluyó temperatura MPU. */
} telemetry_data_t;

/** Último conjunto de telemetría interpretado desde una carga LoRa. */
static telemetry_data_t telemetry;

/** @brief Declaraciones adelantadas de utilidades de dibujo usadas al inicializar. */
static void fill_screen(uint16_t color);
static void draw_rule(int y, uint16_t color);
static void draw_link_footer(void);

/**
 * @brief Devuelve el mapa de bits de cinco columnas para un carácter.
 * @param[in] input Carácter ASCII; las letras se normalizan a mayúsculas.
 * @return Puntero a cinco columnas de bits del glifo o al glifo de interrogación.
 */
static const uint8_t *glyph_for(char input)
{
    static const uint8_t font[][5] = {
        {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
        {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    };
    static const uint8_t blank[5] = {0,0,0,0,0};
    static const uint8_t equal[5] = {0x14,0x14,0x14,0x14,0x14};
    static const uint8_t dot[5] = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t comma[5] = {0x00,0x40,0x20,0x00,0x00};
    static const uint8_t semicolon[5] = {0x00,0x56,0x36,0x00,0x00};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t dash[5] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t slash[5] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t percent[5] = {0x63,0x13,0x08,0x64,0x63};
    static const uint8_t question[5] = {0x02,0x01,0x51,0x09,0x06};

    char c = (char)toupper((unsigned char)input);
    if (c >= 'A' && c <= 'Z') return font[c - 'A'];
    if (c >= '0' && c <= '9') return font[26 + c - '0'];
    switch (c) {
        case ' ': return blank;
        case '=': return equal;
        case '.': return dot;
        case ',': return comma;
        case ';': return semicolon;
        case ':': return colon;
        case '-': return dash;
        case '/': return slash;
        case '%': return percent;
        default: return question;
    }
}

/** @brief Rasteriza un carácter 5x7 en una celda de 6x8 píxeles.
 * @param[in] x Coordenada horizontal de la celda.
 * @param[in] y Coordenada vertical de la celda.
 * @param[in] c Carácter que se dibujará.
 * @param[in] foreground Color RGB565 del glifo.
 * @param[in] background Color RGB565 del fondo.
 */
static void draw_char(int x, int y, char c, uint16_t foreground, uint16_t background)
{
    const uint8_t *glyph = glyph_for(c);
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 6; ++col) {
            bool set = col < 5 && row < 7 && (glyph[col] & (1U << row));
            char_pixels[row * 6 + col] = set ? foreground : background;
        }
    }
    if (x >= 0 && x + 6 <= TFT_WIDTH && y >= 0 && y + 8 <= TFT_HEIGHT) {
        draw_bitmap_sync(x, y, x + 6, y + 8, char_pixels);
    }
}

/** @brief Dibuja texto pequeño sobre el fondo general de pantalla.
 * @param[in] x Coordenada horizontal inicial.
 * @param[in] y Coordenada vertical inicial.
 * @param[in] text Cadena terminada en NUL.
 * @param[in] color Color RGB565 del texto.
 */
static void draw_text(int x, int y, const char *text, uint16_t color)
{
    while (*text && x + 6 <= TFT_WIDTH) {
        draw_char(x, y, *text++, color, COLOR_BG);
        x += 6;
    }
}

/** @brief Dibuja texto pequeño sobre un color de fondo definido.
 * @param[in] x Coordenada horizontal inicial.
 * @param[in] y Coordenada vertical inicial.
 * @param[in] text Cadena terminada en NUL.
 * @param[in] foreground Color RGB565 del texto.
 * @param[in] background Color RGB565 de los píxeles sin tinta.
 */
static void draw_text_on_color(int x, int y, const char *text,
                               uint16_t foreground, uint16_t background)
{
    while (*text && x + 6 <= TFT_WIDTH) {
        draw_char(x, y, *text++, foreground, background);
        x += 6;
    }
}

/** @brief Dibuja un glifo ampliado usando escala entera de 1x o 2x.
 * @param[in] x Coordenada horizontal inicial.
 * @param[in] y Coordenada vertical inicial.
 * @param[in] c Carácter a dibujar.
 * @param[in] color Color RGB565 del glifo.
 * @param[in] scale Factor de ampliación admitido: 1 o 2.
 */
static void draw_char_scaled(int x, int y, char c, uint16_t color, int scale)
{
    const uint8_t *glyph = glyph_for(c);
    const int width = 5 * scale;
    const int height = 7 * scale;
    if (scale > 2 || x < 0 || y < 0 || x + width > TFT_WIDTH || y + height > TFT_HEIGHT) {
        return;
    }
    for (int py = 0; py < height; ++py) {
        for (int px = 0; px < width; ++px) {
            bool set = (glyph[px / scale] & (1U << (py / scale))) != 0;
            large_char_pixels[py * width + px] = set ? color : COLOR_CARD;
        }
    }
    draw_bitmap_sync(x, y, x + width, y + height, large_char_pixels);
}

/** @brief Dibuja una cadena con caracteres ampliados.
 * @param[in] x Coordenada horizontal inicial.
 * @param[in] y Coordenada vertical inicial.
 * @param[in] text Cadena terminada en NUL.
 * @param[in] color Color RGB565 del texto.
 * @param[in] scale Escala entera aplicada a cada glifo.
 */
static void draw_text_scaled(int x, int y, const char *text, uint16_t color, int scale)
{
    while (*text && x + 6 * scale <= TFT_WIDTH) {
        draw_char_scaled(x, y, *text++, color, scale);
        x += 6 * scale;
    }
}

/** @brief Rellena un rectángulo visible, dividiendo el envío en bloques.
 * @param[in] x Coordenada horizontal de la esquina superior izquierda.
 * @param[in] y Coordenada vertical de la esquina superior izquierda.
 * @param[in] width Ancho del rectángulo en píxeles.
 * @param[in] height Alto del rectángulo en píxeles.
 * @param[in] color Color RGB565 de relleno.
 */
static void fill_rect(int x, int y, int width, int height, uint16_t color)
{
    if (width <= 0 || height <= 0 || x < 0 || y < 0 ||
        x + width > TFT_WIDTH || y + height > TFT_HEIGHT) return;
    int rows_per_transfer = (TFT_WIDTH * DRAW_CHUNK) / width;
    if (rows_per_transfer > DRAW_CHUNK) rows_per_transfer = DRAW_CHUNK;
    for (int row = 0; row < rows_per_transfer * width; ++row) fill_chunk[row] = color;
    for (int line = y; line < y + height; line += rows_per_transfer) {
        int rows = y + height - line;
        if (rows > rows_per_transfer) rows = rows_per_transfer;
        draw_bitmap_sync(x, line, x + width, line + rows, fill_chunk);
    }
}

/** @brief Dibuja los cuatro bordes de un rectángulo.
 * @param[in] x Coordenada horizontal de la esquina superior izquierda.
 * @param[in] y Coordenada vertical de la esquina superior izquierda.
 * @param[in] width Ancho en píxeles.
 * @param[in] height Alto en píxeles.
 * @param[in] color Color RGB565 del borde.
 */
static void outline_rect(int x, int y, int width, int height, uint16_t color)
{
    fill_rect(x, y, width, 1, color);
    fill_rect(x, y + height - 1, width, 1, color);
    fill_rect(x, y, 1, height, color);
    fill_rect(x + width - 1, y, 1, height, color);
}

/** @brief Compara una clave de payload con otra ignorando mayúsculas.
 * @param[in] start Inicio de la clave encontrada en el payload.
 * @param[in] length Longitud de la clave encontrada.
 * @param[in] key Clave esperada terminada en NUL.
 * @return true si ambas claves tienen igual longitud y contenido.
 */
static bool key_matches(const char *start, size_t length, const char *key)
{
    if (strlen(key) != length) return false;
    for (size_t i = 0; i < length; ++i) {
        if (toupper((unsigned char)start[i]) != toupper((unsigned char)key[i])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Busca cualquiera de varios alias y convierte su valor numérico.
 * @param[in] payload Cadena con campos `CLAVE:valor` o `CLAVE=valor`.
 * @param[in] keys Lista de nombres aceptados para el dato.
 * @param[in] key_count Número de alias disponibles en la lista.
 * @param[out] value Valor flotante encontrado.
 * @return true si se encontró un alias seguido de un número válido.
 */
static bool find_value(const char *payload, const char *const *keys,
                       size_t key_count, float *value)
{
    const char *cursor = payload;
    while (*cursor) {
        while (isspace((unsigned char)*cursor) || *cursor == ';' || *cursor == ',') ++cursor;
        const char *end_segment = strpbrk(cursor, " \t\r\n;,");
        if (end_segment == NULL) end_segment = cursor + strlen(cursor);
        const char *equals = memchr(cursor, '=', (size_t)(end_segment - cursor));
        const char *colon = memchr(cursor, ':', (size_t)(end_segment - cursor));
        const char *separator = equals != NULL ? equals : colon;
        if (separator != NULL) {
            const char *key_start = cursor;
            while (key_start < separator && isspace((unsigned char)*key_start)) ++key_start;
            const char *key_end = separator;
            while (key_end > key_start && isspace((unsigned char)key_end[-1])) --key_end;
            for (size_t i = 0; i < key_count; ++i) {
                if (key_matches(key_start, (size_t)(key_end - key_start), keys[i])) {
                    char *number_end = NULL;
                    float parsed = strtof(separator + 1, &number_end);
                    if (number_end != separator + 1) {
                        *value = parsed;
                        return true;
                    }
                }
            }
        }
        if (*end_segment == '\0') break;
        cursor = end_segment + 1;
    }
    return false;
}

/** @brief Dibuja el marco y las etiquetas de una tarjeta ambiental.
 * @param[in] x Coordenada horizontal de la esquina superior izquierda.
 * @param[in] y Coordenada vertical de la esquina superior izquierda.
 * @param[in] label Texto identificador del dato.
 * @param[in] unit Texto de la unidad.
 * @param[in] accent Color RGB565 de acento del borde.
 */
static void draw_environment_card(int x, int y, const char *label,
                                 const char *unit, uint16_t accent)
{
    fill_rect(x, y, 108, 54, COLOR_CARD);
    outline_rect(x, y, 108, 54, accent);
    fill_rect(x, y, 108, 2, accent);
    draw_text_on_color(x + 6, y + 6, label, COLOR_DATA, COLOR_CARD);
    draw_text_on_color(x + 78, y + 34, unit, COLOR_DATA, COLOR_CARD);
}

/** @brief Dibuja una tarjeta para un eje de acelerómetro o giroscopio.
 * @param[in] x Coordenada horizontal de la esquina superior izquierda.
 * @param[in] y Coordenada vertical de la esquina superior izquierda.
 * @param[in] axis Nombre del eje (X, Y o Z).
 * @param[in] accent Color RGB565 para distinguir acelerómetro y giroscopio.
 */
static void draw_axis_card(int x, int y, const char *axis, uint16_t accent)
{
    fill_rect(x, y, 108, 38, COLOR_CARD);
    outline_rect(x, y, 108, 38, accent);
    draw_text_on_color(x + 6, y + 6, axis, COLOR_DATA, COLOR_CARD);
}

/** @brief Limpia la pantalla y dibuja marca, estado y encabezado de página.
 * @param[in] section Nombre del subsistema mostrado.
 * @param[in] page_number Indicador ordinal de página.
 */
static void draw_page_header(const char *section, const char *page_number)
{
    fill_screen(COLOR_BG);
    draw_text(9, 8, "ASTRA I", COLOR_CYAN);
    draw_text(174, 8, "GROUND RX", COLOR_GREEN);
    draw_rule(23, COLOR_CYAN);
    draw_text(10, 28, section, COLOR_GRAY);
    draw_text(205, 28, page_number, COLOR_CYAN);
}

/** @brief Traza una línea rasterizada entre dos puntos del búfer histórico.
 * @param[in] x0 Coordenada horizontal del punto inicial.
 * @param[in] y0 Coordenada vertical del punto inicial.
 * @param[in] x1 Coordenada horizontal del punto final.
 * @param[in] y1 Coordenada vertical del punto final.
 */
static void draw_graph_line(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
        if (x0 >= 0 && x0 < GRAPH_WIDTH && y0 >= 0 && y0 < GRAPH_HEIGHT) {
            graph_pixels[y0 * GRAPH_WIDTH + x0] = COLOR_CYAN;
            if (y0 + 1 < GRAPH_HEIGHT) {
                graph_pixels[(y0 + 1) * GRAPH_WIDTH + x0] = COLOR_CYAN;
            }
        }
        if (x0 == x1 && y0 == y1) break;
        int twice_error = 2 * error;
        if (twice_error >= dy) { error += dy; x0 += sx; }
        if (twice_error <= dx) { error += dx; y0 += sy; }
    }
}

/**
 * @brief Agrega una muestra de temperatura a la gráfica circular horizontal.
 * @param[in] temperature Temperatura en grados Celsius, limitada al rango -20..80.
 * @param[in] visible true para transferir el gráfico a la TFT ahora.
 */
static void graph_add_temperature(float temperature, bool visible)
{
    if (temperature < -20.0f) temperature = -20.0f;
    if (temperature > 80.0f) temperature = 80.0f;
    int normalized = (int)(temperature + 20.0f);
    int new_y = GRAPH_HEIGHT - 2 - normalized * (GRAPH_HEIGHT - 4) / 100;

    const unsigned max_samples = (GRAPH_WIDTH - 1) / 6 + 1;
    if (graph_sample_count == 0) {
        draw_graph_line(0, new_y, 0, new_y);
        graph_sample_count = 1;
    } else if (graph_sample_count < max_samples) {
        int previous_x = (int)(graph_sample_count - 1) * 6;
        int next_x = (int)graph_sample_count * 6;
        draw_graph_line(previous_x, graph_last_y, next_x, new_y);
        ++graph_sample_count;
    } else {
        for (int row = 0; row < GRAPH_HEIGHT; ++row) {
            uint16_t *line = &graph_pixels[row * GRAPH_WIDTH];
            memmove(line, line + 6, (GRAPH_WIDTH - 6) * sizeof(uint16_t));
            for (int col = GRAPH_WIDTH - 6; col < GRAPH_WIDTH; ++col) {
                line[col] = COLOR_PLOT;
            }
        }
        draw_graph_line(GRAPH_WIDTH - 10, graph_last_y, GRAPH_WIDTH - 4, new_y);
    }
    graph_last_y = new_y;
    if (visible) {
        draw_bitmap_sync(GRAPH_X, GRAPH_Y, GRAPH_X + GRAPH_WIDTH,
                         GRAPH_Y + GRAPH_HEIGHT, graph_pixels);
    }
}

/** @brief Dibuja el marco fijo de la página de ambiente BME680. */
static void draw_environment_page_frame(void)
{
    draw_page_header("BME680 / AMBIENTE", "1/2");
    draw_environment_card(8, 38, "TEMP", "C", COLOR_YELLOW);
    draw_environment_card(124, 38, "PRESION", "HPA", COLOR_CYAN);
    draw_environment_card(8, 98, "HUMEDAD", "%", COLOR_GREEN);
    draw_environment_card(124, 98, "GAS", "OHM", COLOR_YELLOW);
    draw_text(10, 158, "TEMPERATURA / HISTORIAL", COLOR_CYAN);
    if (graph_sample_count == 0) {
        for (size_t i = 0; i < GRAPH_WIDTH * GRAPH_HEIGHT; ++i) {
            graph_pixels[i] = COLOR_PLOT;
        }
    }
    draw_bitmap_sync(GRAPH_X, GRAPH_Y, GRAPH_X + GRAPH_WIDTH,
                     GRAPH_Y + GRAPH_HEIGHT, graph_pixels);
    draw_rule(291, COLOR_BLUE);
}

/** @brief Dibuja el marco fijo de la página de movimiento MPU6050. */
static void draw_imu_page_frame(void)
{
    draw_page_header("MPU6050 / MOVIMIENTO", "2/2");
    draw_text(10, 43, "ACCEL (G)", COLOR_CYAN);
    draw_text(126, 43, "GYRO (DPS)", COLOR_GREEN);
    draw_axis_card(8, 58, "X", COLOR_CYAN);
    draw_axis_card(124, 58, "X", COLOR_GREEN);
    draw_axis_card(8, 101, "Y", COLOR_CYAN);
    draw_axis_card(124, 101, "Y", COLOR_GREEN);
    draw_axis_card(8, 144, "Z", COLOR_CYAN);
    draw_axis_card(124, 144, "Z", COLOR_GREEN);
    fill_rect(8, 194, 224, 48, COLOR_CARD);
    outline_rect(8, 194, 224, 48, COLOR_YELLOW);
    draw_text_on_color(16, 212, "TEMP MPU", COLOR_DATA, COLOR_CARD);
    draw_rule(291, COLOR_BLUE);
}

/** @brief Actualiza los valores numéricos de las tarjetas BME680. */
static void render_environment_values(void)
{
    char value[16];
    const int xs[] = {8, 124, 8, 124};
    const int ys[] = {38, 38, 98, 98};
    const float values[] = {telemetry.bme_temp, telemetry.pressure,
                            telemetry.humidity, telemetry.gas};
    const bool valid[] = {telemetry.has_bme_temp, telemetry.has_pressure,
                          telemetry.has_humidity, telemetry.has_gas};
    for (int i = 0; i < 4; ++i) {
        fill_rect(xs[i] + 4, ys[i] + 19, 72, 28, COLOR_CARD);
        if (!valid[i]) {
            snprintf(value, sizeof(value), "--");
        } else if (i == 1 || i == 3) {
            snprintf(value, sizeof(value), "%.0f", (double)values[i]);
        } else {
            snprintf(value, sizeof(value), "%.1f", (double)values[i]);
        }
        draw_text_scaled(xs[i] + 5, ys[i] + 21, value, COLOR_DATA, 2);
    }
}

/** @brief Limpia y dibuja un valor de eje con formato según su tipo.
 * @param[in] x Coordenada horizontal base de la tarjeta.
 * @param[in] y Coordenada vertical base de la tarjeta.
 * @param[in] valid Indica si la lectura fue incluida en la telemetría.
 * @param[in] value Valor numérico del eje.
 * @param[in] gyro true para formato de giroscopio; false para aceleración.
 */
static void render_axis_value(int x, int y, bool valid, float value, bool gyro)
{
    char text[16];
    fill_rect(x + 22, y + 14, 82, 22, COLOR_CARD);
    if (valid) snprintf(text, sizeof(text), gyro ? "%.1f" : "%.2f", (double)value);
    else snprintf(text, sizeof(text), "--");
    draw_text_scaled(x + 25, y + 16, text, COLOR_DATA, 2);
}

/** @brief Actualiza aceleraciones, velocidades angulares y temperatura MPU. */
static void render_imu_values(void)
{
    render_axis_value(8, 58, telemetry.has_accel_x, telemetry.accel_x, false);
    render_axis_value(8, 101, telemetry.has_accel_y, telemetry.accel_y, false);
    render_axis_value(8, 144, telemetry.has_accel_z, telemetry.accel_z, false);
    render_axis_value(124, 58, telemetry.has_gyro_x, telemetry.gyro_x, true);
    render_axis_value(124, 101, telemetry.has_gyro_y, telemetry.gyro_y, true);
    render_axis_value(124, 144, telemetry.has_gyro_z, telemetry.gyro_z, true);
    fill_rect(94, 202, 82, 30, COLOR_CARD);
    if (telemetry.has_mpu_temp) {
        char value[12];
        snprintf(value, sizeof(value), "%.1f", (double)telemetry.mpu_temp);
        draw_text_scaled(98, 207, value, COLOR_DATA, 2);
    } else {
        draw_text_scaled(98, 207, "--.-", COLOR_DATA, 2);
    }
}

/** @brief Redibuja el contenido de la página activa y su pie de enlace. */
static void draw_current_page(void)
{
    if (active_page == 0) {
        draw_environment_page_frame();
        render_environment_values();
    } else {
        draw_imu_page_frame();
        render_imu_values();
    }
    draw_link_footer();
}

/** @brief Dibuja contadores RX/perdidos y estado/RSSI del enlace LoRa. */
static void draw_link_footer(void)
{
    char line[48];
    fill_rect(10, 297, 220, 22, COLOR_BG);
    snprintf(line, sizeof(line), "RX %lu  LOST %lu",
             (unsigned long)displayed_received,
             (unsigned long)displayed_lost);
    draw_text(10, 299, line, COLOR_WHITE);
    if (link_online) {
        snprintf(line, sizeof(line), "LINK OK  RSSI %d", displayed_rssi);
        draw_text(10, 310, line, COLOR_GREEN);
    } else {
        draw_text(10, 310, "SIN DATOS - BUSCANDO", COLOR_YELLOW);
    }
}

/** @brief Rellena toda la pantalla con un color RGB565. */
static void fill_screen(uint16_t color)
{
    for (size_t i = 0; i < TFT_WIDTH * DRAW_CHUNK; ++i) fill_chunk[i] = color;
    for (int y = 0; y < TFT_HEIGHT; y += DRAW_CHUNK) {
        draw_bitmap_sync(0, y, TFT_WIDTH, y + DRAW_CHUNK, fill_chunk);
    }
}

/** @brief Dibuja una regla horizontal de dos píxeles de alto.
 * @param[in] y Coordenada vertical de inicio.
 * @param[in] color Color RGB565 de la regla.
 */
static void draw_rule(int y, uint16_t color)
{
    for (size_t i = 0; i < TFT_WIDTH * 2; ++i) fill_chunk[i] = color;
    draw_bitmap_sync(0, y, TFT_WIDTH, y + 2, fill_chunk);
}

/**
 * @brief Inicializa SPI2, el panel ST7789 y los recursos de transferencia.
 * @return ESP_OK al inicializar; error ESP-IDF si SPI, semáforo o panel falla.
 *
 * La orientación aplica espejo horizontal para que la imagen vertical del
 * módulo concreto se lea correctamente. El bus SPI debe estar libre antes.
 */
esp_err_t display_init(void)
{
    color_done_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(color_done_sem != NULL, ESP_ERR_NO_MEM, TAG,
                        "No se pudo crear el semaforo de dibujo");

    spi_bus_config_t bus_config = {
        .sclk_io_num = PIN_SPI_SCK,
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = GRAPH_WIDTH * GRAPH_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO),
                        TAG, "No se pudo inicializar SPI2");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_TFT_DC,
        .cs_gpio_num = PIN_TFT_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        // Profundidad 1: cada pequeno buffer de texto se reutiliza al dibujar el siguiente caracter.
        .trans_queue_depth = 1,
        .on_color_trans_done = color_transfer_done,
        .user_ctx = color_done_sem,
        .flags = { .dc_low_on_data = 0, .quad_mode = 0, .octal_mode = 0,
                   .sio_mode = 0, .lsb_first = 0, .cs_high_active = 0 },
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                                  &io_config, &io),
                        TAG, "No se pudo crear la interfaz SPI de la TFT");

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_TFT_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(io, &panel_config, &panel),
                        TAG, "No se pudo crear el panel ST7789");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "Reset de TFT fallo");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "Init de TFT fallo");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panel, false), TAG, "Rotacion de TFT fallo");
    // La TFT de este modulo necesita swap XY sin espejo para leer el texto normal.
    // En orientacion nativa vertical, este panel requiere invertir el eje X para leer normal.
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(panel, true, false), TAG, "Orientacion de TFT fallo");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), TAG, "TFT no encendio");

    fill_screen(COLOR_BG);
    ESP_LOGI(TAG, "ST7789 lista en SPI2");
    return ESP_OK;
}

/** @brief Restablece datos visuales e indica que el receptor está esperando. */
void display_show_waiting(void)
{
    memset(&telemetry, 0, sizeof(telemetry));
    displayed_received = 0;
    displayed_lost = 0;
    displayed_rssi = 0;
    link_online = false;
    active_page = 0;
    page_started_tick = xTaskGetTickCount();
    draw_current_page();
}

/**
 * @brief Analiza los campos del payload, actualiza telemetría y refresca valores.
 *
 * Los campos reconocidos usan pares `CLAVE:valor` o `CLAVE=valor`; las claves
 * se comparan sin distinguir mayúsculas. Los campos ausentes conservan su
 * último valor, pero no se presentan como inválidos una vez vistos.
 */
void display_show_packet(const lora_packet_t *packet)
{
    float value = 0.0f;
    static const char *const bme_temp_keys[] = {"TBME", "TEMP_BME", "BME_TEMP", "T_BME", "TEMP", "TEMPERATURE", "TEMPERATURA", "T"};
    static const char *const pressure_keys[] = {"PRES", "PRESSURE", "PRESION", "P"};
    static const char *const humidity_keys[] = {"HUM", "HUMIDITY", "HUMEDAD", "H"};
    static const char *const gas_keys[] = {"GAS", "GAS_RES", "GASRES", "GAS_OHM", "RES_GAS"};
    static const char *const accel_x_keys[] = {"AX", "ACCX", "ACC_X", "ACCELX", "ACCEL_X"};
    static const char *const accel_y_keys[] = {"AY", "ACCY", "ACC_Y", "ACCELY", "ACCEL_Y"};
    static const char *const accel_z_keys[] = {"AZ", "ACCZ", "ACC_Z", "ACCELZ", "ACCEL_Z"};
    static const char *const gyro_x_keys[] = {"GX", "GYROX", "GYRO_X"};
    static const char *const gyro_y_keys[] = {"GY", "GYROY", "GYRO_Y"};
    static const char *const gyro_z_keys[] = {"GZ", "GYROZ", "GYRO_Z"};
    static const char *const mpu_temp_keys[] = {"TMPU", "TEMP_MPU", "MPU_TEMP", "T_MPU", "TM"};

#define READ_TELEMETRY(keys, count, field, valid_field) \
    do { \
        if (find_value(packet->payload, keys, count, &value)) { \
            telemetry.field = value; \
            telemetry.valid_field = true; \
        } \
    } while (0)

    READ_TELEMETRY(bme_temp_keys, sizeof(bme_temp_keys) / sizeof(bme_temp_keys[0]), bme_temp, has_bme_temp);
    READ_TELEMETRY(pressure_keys, sizeof(pressure_keys) / sizeof(pressure_keys[0]), pressure, has_pressure);
    READ_TELEMETRY(humidity_keys, sizeof(humidity_keys) / sizeof(humidity_keys[0]), humidity, has_humidity);
    READ_TELEMETRY(gas_keys, sizeof(gas_keys) / sizeof(gas_keys[0]), gas, has_gas);
    READ_TELEMETRY(accel_x_keys, sizeof(accel_x_keys) / sizeof(accel_x_keys[0]), accel_x, has_accel_x);
    READ_TELEMETRY(accel_y_keys, sizeof(accel_y_keys) / sizeof(accel_y_keys[0]), accel_y, has_accel_y);
    READ_TELEMETRY(accel_z_keys, sizeof(accel_z_keys) / sizeof(accel_z_keys[0]), accel_z, has_accel_z);
    READ_TELEMETRY(gyro_x_keys, sizeof(gyro_x_keys) / sizeof(gyro_x_keys[0]), gyro_x, has_gyro_x);
    READ_TELEMETRY(gyro_y_keys, sizeof(gyro_y_keys) / sizeof(gyro_y_keys[0]), gyro_y, has_gyro_y);
    READ_TELEMETRY(gyro_z_keys, sizeof(gyro_z_keys) / sizeof(gyro_z_keys[0]), gyro_z, has_gyro_z);
    READ_TELEMETRY(mpu_temp_keys, sizeof(mpu_temp_keys) / sizeof(mpu_temp_keys[0]), mpu_temp, has_mpu_temp);
#undef READ_TELEMETRY

    displayed_received = packet->packet_count;
    displayed_lost = packet->lost_count;
    displayed_rssi = packet->rssi_dbm;
    link_online = true;
    last_packet_tick = xTaskGetTickCount();

    if (telemetry.has_bme_temp) {
        graph_add_temperature(telemetry.bme_temp, active_page == 0);
    }
    if (active_page == 0) render_environment_values();
    else render_imu_values();
    draw_link_footer();
}

/**
 * @brief Marca el enlace como inactivo tras 3 s y cambia la página cada 5 s.
 *
 * Se invoca periódicamente desde la tarea gráfica, incluso cuando no llegan
 * paquetes, para que el estado de búsqueda y la alternancia sigan avanzando.
 */
void display_check_link_timeout(void)
{
    TickType_t now = xTaskGetTickCount();
    bool status_changed = false;
    if (link_online &&
        (TickType_t)(now - last_packet_tick) >= pdMS_TO_TICKS(3000)) {
        link_online = false;
        status_changed = true;
        ESP_LOGW(TAG, "Sin paquetes durante 3 s; se sigue escuchando LoRa");
    }

    if ((TickType_t)(now - page_started_tick) >= pdMS_TO_TICKS(5000)) {
        active_page = (active_page + 1) % 2;
        page_started_tick = now;
        draw_current_page();
    } else if (status_changed) {
        draw_link_footer();
    }
}
