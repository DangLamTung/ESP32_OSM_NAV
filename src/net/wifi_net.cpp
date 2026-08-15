/**
 * wifi_net.cpp — STA WiFi for network tile fetching.
 *
 * Non-blocking: tapping the WiFi button updates the UI immediately and the
 * actual connect runs in the background. The result arrives via WiFi events
 * (GOT_IP / DISCONNECTED), so the UI loop never blocks for the connect.
 *
 * NOTE: credentials come from app_config.h -> secrets.h (GIT-IGNORED). If
 * secrets.h is missing, the placeholders in app_config.h apply and the connect
 * will fail until real credentials are added to src/secrets.h.
 */
#include "wifi_net.h"
#include "app_config.h"
#include "map_view.h"
#include "ui_controls.h"
#include <WiFi.h>
#include <esp_log.h>

static const char *TAG = "wifi";
static bool s_connected = false;
static bool s_pending   = false;   /* connect requested, awaiting GOT_IP/fail */
static bool s_eventReg  = false;   /* WiFi.onEvent registered once */

static void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        if (s_pending)
        {
            s_pending = false;
            s_connected = true;
            Serial.printf("[wifi] connected, IP=%s\n",
                          WiFi.localIP().toString().c_str());
            /* Missing SD tiles now fetch over the network (TLS via the lazy
             * ReusableTileFetcher). */
            map_set_tile_mode(OpenStreetMap::TILE_AUTO);
            ESP_LOGI(TAG, "tiles -> AUTO (network fallback)");
        }
        ui_mark_redraw();
        break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        if (s_pending)
        {
            s_pending = false;
            Serial.printf("[wifi] connect failed (reason=%d)\n",
                          (int)info.wifi_sta_disconnected.reason);
        }
        if (s_connected)
        {
            s_connected = false;
            map_set_tile_mode(OpenStreetMap::TILE_SD_ONLY);
            ESP_LOGI(TAG, "disconnected, tiles -> SD-only");
        }
        ui_mark_redraw();
        break;

    default:
        break;
    }
}

void wifi_net_connect(void)
{
    /* toggle: on -> off; connecting -> cancel; off -> connect */
    if (s_connected || s_pending)
    {
        wifi_net_disconnect();
        return;
    }
    if (!s_eventReg)
    {
        WiFi.onEvent(onWifiEvent);
        s_eventReg = true;
    }
    s_pending = true;
    Serial.printf("[wifi] connecting to \"%s\"...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);   /* returns immediately; result via event */
    ui_mark_redraw();
}

void wifi_net_disconnect(void)
{
    s_pending = false;
    s_connected = false;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    map_set_tile_mode(OpenStreetMap::TILE_SD_ONLY);
    ESP_LOGI(TAG, "disconnected, tiles -> SD-only");
    ui_mark_redraw();
}

bool wifi_net_connected(void) { return s_connected; }
bool wifi_net_pending(void)   { return s_pending; }

const char *wifi_net_label(void)
{
    if (s_connected) return "WiFi: on";
    if (s_pending)   return "WiFi: ...";
    return "WiFi: off";
}
