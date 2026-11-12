# API del firmware

Versión documentada: **1.0.5**. Las funciones se dividen en cuatro módulos; `main.c` los coordina y contiene la lógica de aplicación.

## `main.c`

| Función | Responsabilidad |
|---|---|
| `app_main()` | Registra hooks idle, prepara GPIO, inicializa TFT, SX1278 y AP, luego crea cola y tareas. |
| `radio_task()` | Sondea RX, valida checksum y campos, actualiza pérdidas, responde ACK y publica el paquete. |
| `display_task()` | Consume la última muestra y actualiza la pantalla y sus temporizadores. |
| `cpu_task()` | Calcula cada segundo la carga estimada y actualiza métricas de pantalla/web. |
| `checksum_ok()` | Valida XOR del contenido entre `$` y `*`. |
| `parse_sensor_frame()` | Convierte la trama `$D,...*XX` a secuencia y once magnitudes. |
| `update_loss()` | Acumula huecos de secuencia y maneja el wrap de 255 a 0. |

## `lora_sx127x.c/.h`

| Función | Responsabilidad |
|---|---|
| `lora_sx127x_init()` | Configura GPIO RESET/DIO0, registra el periférico SPI y programa 433 MHz, SF7, BW125, CR4/5, CRC y sync word `0x12`. |
| `lora_sx127x_receive()` | Lee `RxDone`, descarta errores CRC y extrae payload, RSSI y SNR. Es de sondeo y no espera paquetes. |
| `lora_sx127x_send_ack()` | Transmite `$A,<seq>,OK` y vuelve a RX continuo. |
| `lora_sx127x_retry_receive()` | Limpia IRQ y rearma recepción continua. |

## `display.c/.h`

| Función | Responsabilidad |
|---|---|
| `display_init()` | Inicia SPI2, controlador ST7789, orientación y backlight. |
| `display_show_waiting()` | Dibuja estado inicial sin enlace. |
| `display_show_packet()` | Analiza claves de telemetría, conserva valores conocidos y refresca la interfaz. |
| `display_update_metrics()` | Refresca CPU y porcentaje de pérdida aunque no llegue otra trama. |
| `display_check_link_timeout()` | Actualiza pérdida de enlace y alterna las páginas. |

La TFT y LoRa comparten SPI2. CS de cada módulo debe permanecer separado. El callback de transferencia avisa con un semáforo FreeRTOS cuando DMA termina; así el buffer de dibujo no se reutiliza antes de tiempo.

## `wifi_web.c/.h`

| Función | Responsabilidad |
|---|---|
| `wifi_web_init()` | Arranca NVS, TCP/IP, AP WPA2, HTTP y registra `/` y `/api/data`. |
| `wifi_web_update()` | Copia el paquete más reciente en un snapshot protegido por mutex. |
| `wifi_web_update_metrics()` | Actualiza CPU, pérdidas y errores de la API. |
| `index_handler()` | Sirve el dashboard HTML. |
| `data_handler()` | Devuelve el snapshot JSON y el estado reciente del enlace. |

El JSON expone `online`, `rx`, `lost`, `loss_pct`, `cpu_pct`, `errors`, `rssi`, `snr`, `age_ms` y `payload`.

## Flujo de datos

```text
SX1278 RX → checksum XOR → parseo de secuencia/sensores → ACK LoRa
                                         ├→ cola de longitud 1 → TFT
                                         └→ snapshot mutex → API HTTP → dashboard
Hooks idle CPU0/CPU1 → cpu_task → métricas de CPU y pérdida ───────┘
```

El enlace se considera offline tras 3 segundos en la API web y se indica en la TFT tras 5 segundos sin paquetes. El LED de ausencia de señal parpadea tras el timeout de la tarea de radio.
