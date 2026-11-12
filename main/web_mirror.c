/* Experimental LAN-only M5-SDR screen mirror: 640x360 RGB565 + pointer events. */
#include "web_mirror.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#define MW 640
#define MH 360
#define FRAME_BYTES (MW * MH * 2)
/* Keep framebuffer copies active briefly after the last viewer request. */
#define MIRROR_CLIENT_TIMEOUT_US 3000000LL
static const char *TAG = "mirror";
static uint16_t *s_frame;
static SemaphoreHandle_t s_lock;
static volatile int s_x, s_y, s_down;
static volatile int64_t s_event_us;
static int64_t s_last_frame_us;
static volatile int64_t s_last_client_us;
static httpd_handle_t s_server;
static httpd_handle_t s_touch_server;
static uint16_t *s_previous_frame;
static uint8_t *s_delta_packet;
static bool s_delta_valid;
static uint32_t s_delta_client;
static bool s_enabled = true;

static const char PAGE[] =
"<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>M5-SDR Mirror</title><style>body{margin:0;background:#101622;color:#dce6f2;font:16px sans-serif;text-align:center}canvas{width:min(98vw,1280px);height:auto;touch-action:none;border:1px solid #526070;image-rendering:pixelated}p{margin:12px}small{color:#aab}</style></head><body><p>M5-SDR WEB MIRROR <small id=\"status\">connecting...</small></p><canvas id=\"screen\" width=\"640\" height=\"360\"></canvas><p><small>DELTA FULL 640x360 - mouse / touch controls Tab5</small></p><script>\n"
"const c=document.getElementById('screen'),ctx=c.getContext('2d'),st=document.getElementById('status');\n"
"let busy=false,down=false,frames=0,stamp=performance.now(),bytes=0,client=(Math.random()*0x7fffffff|0)+1,reset=true;\n"
"const fullImage=ctx.createImageData(640,360);\n"
"function rgb(b,offset,d,j){const v=b[offset]|(b[offset+1]<<8);d[j]=(v>>11&31)*255/31;d[j+1]=(v>>5&63)*255/63;d[j+2]=(v&31)*255/31;d[j+3]=255;}\n"
"function drawDelta(b){if(b.length<8||b[0]!==68||b[1]!==76||b[2]!==84||b[3]!==49)throw Error('delta header');const count=b[4]|b[5]<<8;let off=8;for(let t=0;t<count;t++){if(off+8>b.length)throw Error('tile header');const x=b[off]|b[off+1]<<8,y=b[off+2]|b[off+3]<<8,w=b[off+4]|b[off+5]<<8,h=b[off+6]|b[off+7]<<8;off+=8;if(!w||!h||x+w>640||y+h>360||off+w*h*2>b.length)throw Error('tile data');for(let yy=0;yy<h;yy++)for(let xx=0;xx<w;xx++){const j=((y+yy)*640+x+xx)*4;rgb(b,off+(yy*w+xx)*2,fullImage.data,j);}ctx.putImageData(fullImage,0,0,x,y,w,h);off+=w*h*2;}if(off!==b.length)throw Error('delta length');}\n"
"async function frame(){if(busy)return;busy=true;try{const r=await fetch('/delta?client='+client+'&reset='+(reset?1:0),{cache:'no-store'});if(!r.ok)throw Error(r.status);const b=new Uint8Array(await r.arrayBuffer());bytes+=b.length;drawDelta(b);reset=false;frames++;const now=performance.now();if(now-stamp>=2000){const sec=(now-stamp)/1000;st.textContent='LIVE '+(frames/sec).toFixed(1)+' FPS | '+(bytes/1024/sec).toFixed(1)+' KiB/s';frames=0;bytes=0;stamp=now;}}catch(e){reset=true;st.textContent='offline: '+e.message;}finally{busy=false;setTimeout(frame,30);}}\n"
"frame();\n"
"let touchQueue=Promise.resolve(),lastX=0,lastY=0,pressAt=0,lastMove=0;\n"
"function coords(e){const r=c.getBoundingClientRect();lastX=Math.max(0,Math.min(639,Math.floor((e.clientX-r.left)*640/r.width)));lastY=Math.max(0,Math.min(359,Math.floor((e.clientY-r.top)*360/r.height)));}\n"
"function send(d){const x=lastX,y=lastY;touchQueue=touchQueue.catch(()=>{}).then(()=>fetch('http://'+location.hostname+':81/touch?x='+x+'&y='+y+'&d='+d,{method:'POST',mode:'cors',cache:'no-store'}));}\n"
"c.addEventListener('pointerdown',e=>{e.preventDefault();coords(e);down=true;pressAt=performance.now();c.setPointerCapture(e.pointerId);send(1);});\n"
"c.addEventListener('pointermove',e=>{if(!down)return;e.preventDefault();coords(e);const now=performance.now();if(now-lastMove>90){lastMove=now;send(1);}});\n"
"function release(e){if(!down)return;down=false;coords(e);const wait=Math.max(0,100-(performance.now()-pressAt));setTimeout(()=>send(0),wait);}\n"
"c.addEventListener('pointerup',release);c.addEventListener('pointercancel',release);setInterval(()=>{if(down)send(1);},350);\n"
"</script></body></html>"
;
static esp_err_t page_get(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}
/* One viewer at a time: client token forces a complete resync on reconnect. */
static esp_err_t delta_get(httpd_req_t *req) {
    if (!s_frame || !s_previous_frame || !s_delta_packet || !s_lock)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no delta buffer");
    char query[96] = {0};
    unsigned client = 0, reset = 0;
    if (httpd_req_get_url_query_str(req, query, sizeof query) != ESP_OK ||
        sscanf(query, "client=%u&reset=%u", &client, &reset) != 2 || client == 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid client");
    /* A /delta request is the viewer heartbeat; no separate polling task. */
    s_last_client_us = esp_timer_get_time();
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(30)) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "busy");
    }
    bool full = !s_delta_valid || reset != 0 || client != s_delta_client;
    size_t out = 8;
    unsigned count = 0;
    s_delta_packet[0] = 'D'; s_delta_packet[1] = 'L';
    s_delta_packet[2] = 'T'; s_delta_packet[3] = '1';
    s_delta_packet[6] = 0; s_delta_packet[7] = 0;
    /* 20 columns x 12 rows, last row is only 8 pixels tall. */
    for (int ty = 0; ty < MH; ty += 32) {
        int h = MH - ty < 32 ? MH - ty : 32;
        for (int tx = 0; tx < MW; tx += 32) {
            bool changed = full;
            if (!changed) {
                for (int yy = 0; yy < h; yy++) {
                    const uint16_t *a = s_frame + (ty + yy) * MW + tx;
                    const uint16_t *b = s_previous_frame + (ty + yy) * MW + tx;
                    if (memcmp(a, b, 32 * 2) != 0) { changed = true; break; }
                }
            }
            if (!changed) continue;
            size_t need = 8 + (size_t)32 * h * 2;
            if (out + need > FRAME_BYTES + 8 + 240 * 8) {
                xSemaphoreGive(s_lock);
                return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "packet overflow");
            }
            s_delta_packet[out++] = (uint8_t)tx;
            s_delta_packet[out++] = (uint8_t)(tx >> 8);
            s_delta_packet[out++] = (uint8_t)ty;
            s_delta_packet[out++] = (uint8_t)(ty >> 8);
            s_delta_packet[out++] = 32; s_delta_packet[out++] = 0;
            s_delta_packet[out++] = (uint8_t)h; s_delta_packet[out++] = 0;
            for (int yy = 0; yy < h; yy++) {
                const uint16_t *src = s_frame + (ty + yy) * MW + tx;
                memcpy(s_delta_packet + out, src, 64);
                memcpy(s_previous_frame + (ty + yy) * MW + tx, src, 64);
                out += 64;
            }
            count++;
        }
    }
    s_delta_packet[4] = (uint8_t)count;
    s_delta_packet[5] = (uint8_t)(count >> 8);
    s_delta_valid = true;
    s_delta_client = client;
    xSemaphoreGive(s_lock);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(req, (const char *)s_delta_packet, out);
    if (err != ESP_OK) s_delta_valid = false;
    return err;
}
static esp_err_t touch_post(httpd_req_t *req) {
    char q[96]; int x, y, d;
    if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK ||
        sscanf(q, "x=%d&y=%d&d=%d", &x, &y, &d) != 3 ||
        x < 0 || x >= MW || y < 0 || y >= MH || (d != 0 && d != 1))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid touch");
    s_x = x; s_y = y; s_down = d; s_event_us = esp_timer_get_time();
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "", 0);
}
void web_mirror_start(void) {
    if (!s_enabled || s_server) return;
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_frame) s_frame = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_previous_frame) s_previous_frame = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_delta_packet) s_delta_packet = heap_caps_malloc(FRAME_BYTES + 8 + 240 * 8, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_lock || !s_frame || !s_previous_frame || !s_delta_packet) { ESP_LOGE(TAG, "No memory for mirror frame"); return; }
    memset(s_frame, 0, FRAME_BYTES);
    s_delta_valid = false;
    s_last_client_us = 0;
    s_last_frame_us = 0;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.stack_size = 6144;
    cfg.lru_purge_enable = true;
    cfg.send_wait_timeout = 3;
    if (httpd_start(&s_server, &cfg) != ESP_OK) { ESP_LOGE(TAG, "HTTP start failed"); return; }
    httpd_uri_t page = {.uri="/", .method=HTTP_GET, .handler=page_get};
    httpd_uri_t delta = {.uri="/delta", .method=HTTP_GET, .handler=delta_get};
    httpd_uri_t touch = {.uri="/touch", .method=HTTP_POST, .handler=touch_post};
    httpd_register_uri_handler(s_server, &page);
    httpd_register_uri_handler(s_server, &delta);
    httpd_config_t touch_cfg = HTTPD_DEFAULT_CONFIG();
    touch_cfg.server_port = 81;
    touch_cfg.ctrl_port = cfg.ctrl_port + 1;
    touch_cfg.stack_size = 4096;
    touch_cfg.lru_purge_enable = true;
    if (httpd_start(&s_touch_server, &touch_cfg) == ESP_OK) {
        httpd_register_uri_handler(s_touch_server, &touch);
    } else {
        ESP_LOGE(TAG, "Touch server port 81 failed");
    }
    ESP_LOGI(TAG, "Web Mirror port 80, touch port 81 (LAN only; no authentication)");
}
void web_mirror_publish(const uint16_t *pixels) {
    if (!s_enabled || !s_server || !s_frame || !pixels || !s_lock) return;
    int64_t now = esp_timer_get_time();
    /* Without a recent /delta request, avoid the 450 KiB framebuffer copy. */
    int64_t last_client = s_last_client_us;
    if (!last_client || now - last_client > MIRROR_CLIENT_TIMEOUT_US) return;
    if (now - s_last_frame_us < 100000) return;
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) return;
    memcpy(s_frame, pixels, FRAME_BYTES);
    s_last_frame_us = now;
    xSemaphoreGive(s_lock);
}
bool web_mirror_pointer(int *x, int *y, bool *down) {
    if (!s_enabled || !s_touch_server || !s_event_us || esp_timer_get_time() - s_event_us > 1200000) return false;
    *x = s_x; *y = s_y; *down = (s_down != 0);
    return true;
}

/* Persistent enable switch; called from the UI task, never from HTTP handlers. */
bool web_mirror_enabled(void) { return s_enabled; }
void web_mirror_load_settings(void) {
    nvs_handle_t h;
    uint8_t enabled = 1;
    if (nvs_open("station", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "web_enable", &enabled);
        nvs_close(h);
    }
    s_enabled = enabled != 0;
}
void web_mirror_set_enabled(bool enabled) {
    if (enabled == s_enabled) return;
    if (!enabled) {
        s_enabled = false;
        s_event_us = 0;
        s_last_client_us = 0;
        if (s_touch_server) { httpd_stop(s_touch_server); s_touch_server = NULL; }
        if (s_server) { httpd_stop(s_server); s_server = NULL; }
        ESP_LOGI(TAG, "Web Mirror stopped");
    } else {
        s_enabled = true;
        web_mirror_start();
    }
    nvs_handle_t h;
    if (nvs_open("station", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "web_enable", enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}
