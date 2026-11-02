/** @file wifi_web.c @brief AP Wi-Fi, API JSON y panel web con gráficas. */
#include "wifi_web.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/** Nombre de la red local creada por el receptor. */
#define WIFI_AP_SSID     "ASTRA-I-RX"
/** Clave WPA2 del punto de acceso; cámbiese antes de un despliegue público. */
#define WIFI_AP_PASSWORD "ASTRA-I-2026"

/** Etiqueta de registro del subsistema Wi-Fi/HTTP. */
static const char *TAG = "wifi_web";
/**
 * @brief Página completa del panel servida desde flash como HTML estático.
 *
 * Incluye la presentación responsive, tarjetas BME680/MPU6050, gráficas
 * Canvas con hasta 60 muestras recientes y JavaScript que consulta /api/data
 * una vez por segundo. Los alias de claves deben mantenerse compatibles con
 * el analizador de telemetría de display.c.
 */
static const char INDEX_HTML[] =
"<!doctype html><html lang='es'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ASTRA I | Telemetria</title><style>"
"*{box-sizing:border-box}body{margin:0;background:#06101d;color:#eaf7ff;font:16px system-ui,sans-serif}"
"main{max-width:760px;margin:auto;padding:22px}header{display:flex;justify-content:space-between;align-items:center;border-bottom:2px solid #00d9ff;padding-bottom:14px}"
"h1{font-size:22px;margin:0;color:#00d9ff;letter-spacing:2px}small{color:#85a8bd}"
".status{margin:18px 0;padding:14px;border:1px solid #17465a;border-radius:12px;background:#0b1b2b}"
".ok{color:#39efad}.bad{color:#ffcc55}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin:12px 0 22px}"
".card{background:#0b1b2b;border:1px solid #17465a;border-radius:12px;padding:14px;min-height:88px}"
".chart{background:#0b1b2b;border:1px solid #17465a;border-radius:12px;padding:12px;margin:10px 0}.chart canvas{display:block;width:100%;height:150px}"
".label{color:#79a6bf;font-size:12px;text-transform:uppercase;letter-spacing:1px}.value{font-size:25px;font-weight:700;color:#00d9ff;margin-top:7px}"
".unit{font-size:13px;color:#85a8bd;margin-left:5px}h2{font-size:14px;letter-spacing:2px;color:#b7d5e5;margin:22px 0 8px}footer{color:#6f91a4;font-size:12px;margin-top:22px}"
"</style></head><body><main><header><h1>ASTRA I</h1><small>ESTACION RECEPTORA</small></header>"
"<section class='status'><b id='link' class='bad'>SIN DATOS - BUSCANDO</b><div id='meta'>Conectando con el receptor...</div></section>"
"<h2>BME680 / AMBIENTE</h2><section class='grid'>"
"<article class='card'><div class='label'>Temperatura</div><div class='value'><span id='temp'>--</span><span class='unit'>C</span></div></article>"
"<article class='card'><div class='label'>Presion</div><div class='value'><span id='pres'>--</span><span class='unit'>hPa</span></div></article>"
"<article class='card'><div class='label'>Humedad</div><div class='value'><span id='hum'>--</span><span class='unit'>%</span></div></article>"
"<article class='card'><div class='label'>Resistencia de gas</div><div class='value'><span id='gas'>--</span><span class='unit'>ohm</span></div></article></section>"
"<h2>HISTORIAL AMBIENTAL</h2><section class='chart'><div class='label'>Temperatura (C)</div><canvas id='tempChart'></canvas></section>"
"<section class='chart'><div class='label'>Presion (hPa)</div><canvas id='presChart'></canvas></section>"
"<section class='chart'><div class='label'>Humedad (%)</div><canvas id='humChart'></canvas></section>"
"<h2>MPU6050 / MOVIMIENTO</h2><section class='grid'>"
"<article class='card'><div class='label'>Aceleracion X</div><div class='value'><span id='ax'>--</span><span class='unit'>g</span></div></article>"
"<article class='card'><div class='label'>Aceleracion Y</div><div class='value'><span id='ay'>--</span><span class='unit'>g</span></div></article>"
"<article class='card'><div class='label'>Aceleracion Z</div><div class='value'><span id='az'>--</span><span class='unit'>g</span></div></article>"
"<article class='card'><div class='label'>Giroscopio X</div><div class='value'><span id='gx'>--</span><span class='unit'>DPS</span></div></article>"
"<article class='card'><div class='label'>Giroscopio Y</div><div class='value'><span id='gy'>--</span><span class='unit'>DPS</span></div></article>"
"<article class='card'><div class='label'>Giroscopio Z</div><div class='value'><span id='gz'>--</span><span class='unit'>DPS</span></div></article>"
"<article class='card'><div class='label'>Temperatura MPU</div><div class='value'><span id='tm'>--</span><span class='unit'>C</span></div></article></section>"
"<h2>ENLACE LORA</h2><section class='grid'>"
"<article class='card'><div class='label'>Paquetes recibidos</div><div class='value' id='rx'>0</div></article>"
"<article class='card'><div class='label'>Paquetes perdidos</div><div class='value' id='lost'>0</div></article>"
"<article class='card'><div class='label'>RSSI</div><div class='value'><span id='rssi'>--</span><span class='unit'>dBm</span></div></article>"
"<article class='card'><div class='label'>SNR</div><div class='value'><span id='snr'>--</span><span class='unit'>dB</span></div></article></section>"
"<footer>Actualizacion automatica cada segundo. Los datos aparecen cuando el transmisor los incluye en el paquete.</footer></main>"
"<script>/* Alias de nombres de campo aceptados para cada magnitud transmitida. */const aliases={temp:['TBME','TEMP_BME','BME_TEMP','T_BME','TEMP','TEMPERATURE','TEMPERATURA','T'],pres:['PRES','PRESSURE','PRESION','P'],hum:['HUM','HUMIDITY','HUMEDAD','H'],gas:['GAS','GAS_RES','GASRES','GAS_OHM','RES_GAS'],ax:['AX','ACCX','ACC_X','ACCELX','ACCEL_X'],ay:['AY','ACCY','ACC_Y','ACCELY','ACCEL_Y'],az:['AZ','ACCZ','ACC_Z','ACCELZ','ACCEL_Z'],gx:['GX','GYROX','GYRO_X'],gy:['GY','GYROY','GYRO_Y'],gz:['GZ','GYROZ','GYRO_Z'],tm:['TMPU','TEMP_MPU','MPU_TEMP','T_MPU','TM']};"
"/* Separa campos KEY:value/KEY=value y conserva valores numéricos. */function parsePayload(s){const d={};for(const t of s.split(/[\\s;,]+/)){const i=Math.max(t.indexOf(':'),t.indexOf('='));if(i>0){const m=t.slice(i+1).match(/^[-+]?(?:\\d+\\.?\\d*|\\.\\d+)/);if(m)d[t.slice(0,i).toUpperCase()]=m[0]}}return d}"
"const history={temp:[],pres:[],hum:[]};let lastRx=-1;"
"/* Traza una serie en Canvas, escalando Y al rango de las muestras visibles. */function chart(id,values,color){const c=document.getElementById(id),r=c.getBoundingClientRect(),dpr=window.devicePixelRatio||1;if(r.width<10)return;c.width=r.width*dpr;c.height=r.height*dpr;const g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);const w=r.width,h=r.height,L=38,R=8,T=8,B=20,pw=w-L-R,ph=h-T-B;g.clearRect(0,0,w,h);g.strokeStyle='#18384b';g.lineWidth=1;for(let j=0;j<4;j++){const y=T+ph*j/3;g.beginPath();g.moveTo(L,y);g.lineTo(w-R,y);g.stroke()}const a=values.filter(v=>v!==null);if(!a.length){g.fillStyle='#7895a8';g.fillText('Esperando muestras...',L,T+ph/2);return}let lo=Math.min(...a),hi=Math.max(...a);if(hi-lo<0.1){lo-=0.5;hi+=0.5}else{const pad=(hi-lo)*0.12;lo-=pad;hi+=pad}g.fillStyle='#91adbd';g.font='11px system-ui';g.fillText(hi.toFixed(1),2,T+4);g.fillText(lo.toFixed(1),2,T+ph);g.strokeStyle=color;g.lineWidth=2.5;g.beginPath();let active=false;values.forEach((v,i)=>{if(v===null){active=false;return}const x=L+(values.length<2?0:i*pw/(values.length-1)),y=T+ph-(v-lo)*ph/(hi-lo);if(!active){g.moveTo(x,y);active=true}else g.lineTo(x,y)});g.stroke();g.fillStyle='#7895a8';g.fillText('ultimas '+values.length+' muestras',L,h-4)}"
"/* Consulta la API y actualiza valores, estado del enlace e historial ambiental. */async function refresh(){try{const r=await fetch('/api/data',{cache:'no-store'});const d=await r.json();const p=parsePayload(d.payload||'');for(const id in aliases){const k=aliases[id].find(x=>p[x]!==undefined);document.getElementById(id).textContent=k?p[k]:'--'}"
"document.getElementById('rx').textContent=d.rx;document.getElementById('lost').textContent=d.lost;document.getElementById('rssi').textContent=typeof d.rssi==='number'?d.rssi:'--';document.getElementById('snr').textContent=typeof d.snr==='number'?d.snr:'--';"
"if(d.rx!==lastRx){lastRx=d.rx;for(const id of ['temp','pres','hum']){const key=aliases[id].find(k=>p[k]!==undefined);history[id].push(key?Number(p[key]):null);if(history[id].length>60)history[id].shift()}}chart('tempChart',history.temp,'#00d9ff');chart('presChart',history.pres,'#ba8cff');chart('humChart',history.hum,'#39efad');"
"const l=document.getElementById('link');l.textContent=d.online?'ENLACE ACTIVO':'SIN DATOS - BUSCANDO';l.className=d.online?'ok':'bad';document.getElementById('meta').textContent=d.online?`Paquete ${d.rx} · ultimo hace ${d.age_ms} ms`:'El receptor sigue escuchando LoRa';}catch(e){const l=document.getElementById('link');l.textContent='REINTENTANDO';l.className='bad'}}setInterval(refresh,1000);refresh();</script></body></html>";

/** Protege el snapshot compartido entre la tarea de radio y el servidor HTTP. */
static SemaphoreHandle_t snapshot_mutex;
/** Último paquete LoRa publicado para los clientes web. */
static lora_packet_t latest_packet;
/** Indica si el snapshot ya contiene al menos un paquete recibido. */
static bool has_packet;
/** Tick del último paquete, usado para informar edad y estado online. */
static TickType_t latest_packet_tick;
/** Manejador del servidor HTTP embebido de ESP-IDF. */
static httpd_handle_t http_server;

/** @brief Responde GET / con la aplicación web estática del monitor.
 * @param[in,out] request Solicitud HTTP recibida por ESP-IDF.
 * @return Resultado de enviar el documento HTML.
 */
static esp_err_t index_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief Responde GET /api/data con métricas y carga útil serializadas en JSON.
 * @param[in,out] request Solicitud HTTP recibida por ESP-IDF.
 * @return Resultado de enviar JSON o error HTTP 500 si excede el búfer.
 *
 * Copia el snapshot bajo mutex y hace el resto del trabajo sobre una copia
 * local para no retener el bloqueo mientras codifica o envía la respuesta.
 */
static esp_err_t data_handler(httpd_req_t *request)
{
    lora_packet_t snapshot = {0};
    bool valid = false;
    TickType_t tick = 0;
    if (xSemaphoreTake(snapshot_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        snapshot = latest_packet;
        valid = has_packet;
        tick = latest_packet_tick;
        xSemaphoreGive(snapshot_mutex);
    }

    uint32_t age_ms = valid
        ? (uint32_t)(xTaskGetTickCount() - tick) * portTICK_PERIOD_MS
        : UINT32_MAX;
    bool online = valid && age_ms < 3000;
    char escaped_payload[384];
    size_t used = 0;
    if (valid) {
        for (size_t i = 0; i < snapshot.length && used + 2 < sizeof(escaped_payload); ++i) {
            unsigned char c = (unsigned char)snapshot.payload[i];
            if (c == '"' || c == '\\') escaped_payload[used++] = '\\';
            if (c >= 0x20) escaped_payload[used++] = (char)c;
        }
    }
    escaped_payload[used] = '\0';

    char json[640];
    int length = snprintf(json, sizeof(json),
                          "{\"online\":%s,\"rx\":%lu,\"lost\":%lu,\"rssi\":%d,\"snr\":%.1f,\"age_ms\":%lu,\"payload\":\"%s\"}",
                          online ? "true" : "false",
                          (unsigned long)snapshot.packet_count,
                          (unsigned long)snapshot.lost_count,
                          valid ? snapshot.rssi_dbm : 0,
                          valid ? (double)snapshot.snr_db : 0.0,
                          (unsigned long)age_ms,
                          escaped_payload);
    if (length < 0 || length >= sizeof(json)) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "JSON response too large");
    }
    httpd_resp_set_type(request, "application/json; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, json, length);
}

/**
 * @brief Inicializa NVS, TCP/IP, el AP WPA2 y el servidor web local.
 * @return ESP_OK si se registraron las rutas; error ESP-IDF ante un fallo.
 *
 * Rutas disponibles: `/` sirve el panel y `/api/data` entrega el snapshot JSON.
 */
esp_err_t wifi_web_init(void)
{
    snapshot_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(snapshot_mutex != NULL, ESP_ERR_NO_MEM, TAG,
                        "No se pudo crear mutex de datos web");

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) return err;
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "No se pudo inicializar TCP/IP");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG,
                        "No se pudo crear el event loop");
    if (esp_netif_create_default_wifi_ap() == NULL) {
        return ESP_FAIL;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init), TAG, "No se pudo iniciar Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG,
                        "No se pudo seleccionar modo punto de acceso");

    wifi_config_t ap_config = {0};
    memcpy(ap_config.ap.ssid, WIFI_AP_SSID, sizeof(WIFI_AP_SSID) - 1);
    memcpy(ap_config.ap.password, WIFI_AP_PASSWORD, sizeof(WIFI_AP_PASSWORD) - 1);
    ap_config.ap.ssid_len = sizeof(WIFI_AP_SSID) - 1;
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config), TAG,
                        "No se pudo configurar punto de acceso");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "No se pudo encender Wi-Fi");

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.max_uri_handlers = 4;
    ESP_RETURN_ON_ERROR(httpd_start(&http_server, &server_config), TAG,
                        "No se pudo iniciar servidor web");

    const httpd_uri_t index_uri = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL,
    };
    const httpd_uri_t data_uri = {
        .uri = "/api/data", .method = HTTP_GET, .handler = data_handler, .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &index_uri), TAG,
                        "No se pudo registrar pagina principal");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(http_server, &data_uri), TAG,
                        "No se pudo registrar API de telemetria");

    ESP_LOGI(TAG, "Punto de acceso: %s / clave: %s", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "Panel web local: http://192.168.4.1");
    return ESP_OK;
}

/**
 * @brief Copia el paquete más reciente al snapshot consultado por HTTP.
 *
 * El mutex tiene espera acotada para que la tarea de radio no se bloquee
 * demasiado tiempo si el servidor está copiando simultáneamente el snapshot.
 */
void wifi_web_update(const lora_packet_t *packet)
{
    if (packet == NULL || snapshot_mutex == NULL) return;
    if (xSemaphoreTake(snapshot_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        latest_packet = *packet;
        latest_packet_tick = xTaskGetTickCount();
        has_packet = true;
        xSemaphoreGive(snapshot_mutex);
    }
}
