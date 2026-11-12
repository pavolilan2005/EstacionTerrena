# Referencia de módulos y funciones

Esta referencia se genera con Doxygen. El panorama de arquitectura, conexiones, protocolo LoRa, interfaz web y compilación se describe en [README_ADAPTACION.md](../README_ADAPTACION.md).

## Módulos

- `main.c`: inicialización, validación/checksum, cálculo de pérdidas, formateo de la muestra, estimación de CPU y tareas FreeRTOS.
- `lora_sx127x.c/.h`: acceso a registros y FIFO del SX127x, configuración de radio, recepción y ACK.
- `display.c/.h`: inicialización ST7789 y presentación de telemetría y métricas.
- `wifi_web.c/.h`: AP WPA2, servidor HTTP, HTML del dashboard y API JSON protegida por mutex.

## Generar la referencia HTML

Instala Doxygen y, desde la carpeta del proyecto, ejecuta:

```powershell
doxygen Doxyfile
```

Abre `docs/doxygen/html/index.html`. La página inicial de Doxygen es este README de documentación; los comentarios `@brief`, `@param`, `@return` y `@file` de los fuentes generan las páginas de funciones y módulos.
