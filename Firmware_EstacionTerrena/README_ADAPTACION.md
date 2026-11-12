# Estación terrena ASTRA I — Firmware ESP-IDF

**Versión del firmware: 1.0.5**. Proyecto para ESP32 clásico (ESP32 DevKit V1), con receptor SX1278, pantalla ST7789, indicadores de estado y panel web local. La aplicación está organizada en tareas FreeRTOS.

## Funciones

- Recibe por LoRa la telemetría enviada por el nodo 2.
- Comprueba la trama ASCII y su checksum XOR antes de aceptar los datos.
- Calcula pérdidas a partir del contador de secuencia de 8 bits.
- Envía al nodo transmisor el ACK `$A,<secuencia>,OK`.
- Presenta telemetría, RSSI, SNR, paquetes perdidos y CPU en la TFT.
- Crea un punto de acceso Wi-Fi con una página web y una API JSON.

## Mapa de conexiones

### SX1278

| Señal del módulo | GPIO ESP32 |
|---|---:|
| NSS / CS | 5 |
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| RESET | 14 |
| DIO0 | 26 |
| Alimentación | 3V3 |
| Tierra | GND común |

DIO1–DIO5 no se usan. En especial, **SCK no debe conectarse a DIO5**. El bus SPI se comparte con la TFT, por lo que cada periférico debe conservar su propio NSS/CS.

La revisión de la PCB encontró las señales LoRa desplazadas entre pads: revisa continuidad de NSS, MOSI, MISO y SCK en el módulo y aísla las pistas incorrectas antes de cablear puentes. Con la placa apagada, confirma GPIO5→NSS, GPIO23→MOSI, GPIO19→MISO y GPIO18→SCK, sin cortos entre ellas.

### TFT ST7789 y LEDs

| Señal | GPIO ESP32 |
|---|---:|
| SCK / MOSI (SPI compartido) | 18 / 23 |
| CS / DC / RESET | 12 / 2 / 4 |
| Backlight | 27 |
| LED sin señal / LED enlace / LED error | 21 / 22 / 25 |

El MISO de la TFT no se utiliza. Conecta todas las tierras entre módulos. Verifica continuidad y ausencia de cortos entre NSS, SCK, MISO y MOSI, especialmente si reparaste pistas de la PCB.

## Trama y unidades

Formato que transmite el nodo 2:

```text
$D,seq,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps,mpu_centiC,pres_Pa,bme_centiC,hum_milliPct,gas_ohm*XX
```

`XX` es el XOR de los caracteres entre `$` y `*`, codificado como dos dígitos hexadecimales. La secuencia está en el rango 0–255 y se permite que desborde de 255 a 0.

| Campos | Unidad transmitida | Conversión mostrada |
|---|---|---|
| `ax`, `ay`, `az` | mg | g (dividir entre 1000) |
| `gx`, `gy`, `gz` | mdps | grados/s (dividir entre 1000) |
| `mpu_centiC`, `bme_centiC` | centi-°C | °C (dividir entre 100) |
| `pres_Pa` | Pa | hPa (dividir entre 100) |
| `hum_milliPct` | milésimas de % | % (dividir entre 1000) |
| `gas_ohm` | ohmios | ohmios |

El receptor acusa solo tramas válidas con `$A,<seq>,OK` seguido de CR/LF. Ambos nodos deben usar el mismo perfil de radio: **433 MHz, SF7, BW125 kHz, CR 4/5, CRC y sync word `0x12`**.

## Tareas FreeRTOS

- **`lora_rx` (núcleo 1):** sondea DIO0, valida y procesa paquetes, actualiza LEDs, responde ACK y publica una copia para la web y otra para la pantalla.
- **`tft_ui` (núcleo 1):** toma la muestra más reciente de una cola de longitud uno y refresca valores, métricas y estado del enlace.
- **`cpu_stats` (núcleo 0):** cada segundo estima carga con contadores de los idle hooks y publica métricas.
- **Servidor HTTP:** ofrece la página principal y `/api/data`; un mutex protege la copia de telemetría.

La carga de CPU es una estimación basada en ticks de idle y no sustituye al perfilado de tiempo de ejecución. El porcentaje de pérdida se calcula a partir de secuencias observadas desde el arranque.

## LEDs

- **GPIO21, sin señal:** se apaga al recibir una trama válida y parpadea cuando vence el tiempo de enlace.
- **GPIO22, enlace:** se enciende al recibir una trama válida.
- **GPIO25, error:** pulso ante una trama inválida o queda encendido si falla el ACK; el siguiente ACK correcto lo apaga.

## Panel web

Conéctate al AP **`ASTRA-I-RX`**, clave **`ASTRA-I-2026`**, y abre `http://192.168.4.1`.

- `GET /`: aplicación web de telemetría.
- `GET /api/data`: JSON con `online`, `rx`, `lost`, `loss_pct`, `cpu_pct`, `errors`, `rssi`, `snr`, `age_ms` y `payload`.

El AP está pensado para pruebas locales. Cambia la clave antes de usar el equipo fuera de un entorno de laboratorio.

## Compilar y cargar

Requisitos: ESP-IDF 6.1 configurado para ESP32, Python y herramientas Xtensa instaladas por ESP-IDF.

```powershell
cd D:\Proyectos\EstacionTerrena\Firmware_EstacionTerrena
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

Cambia `COMx` por el puerto serie real. Para limpiar una configuración de compilación previa, ejecuta `idf.py fullclean` y configura de nuevo el target.

## Documentación API

El código contiene comentarios Doxygen en español. Desde la raíz del proyecto genera la referencia HTML con:

```powershell
doxygen Doxyfile
```

La salida queda en `docs/doxygen/html/index.html`. La documentación funcional y de conexiones está en este archivo.

## Versionado

El número vigente se indica en el comentario inicial de `main/main.c`, en el `project(... VERSION ...)` de `CMakeLists.txt` y en `Doxyfile`. Incrementa el PATCH en cada modificación del firmware y actualiza este documento.


