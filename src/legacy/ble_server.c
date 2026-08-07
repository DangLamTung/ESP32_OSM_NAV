/**
 * ble_server.c — BLE GATT server (NimBLE) that receives map XML packets
 * from the phone and renders them via map_render_show().
 *
 * GATT layout (128-bit custom UUIDs):
 *   Service 5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c
 *     Char   5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c  (Write + WriteNoResp)
 *
 * Protocol: the phone writes the full compact map XML (any chunking), and
 * the packet is finalized when `</map>` is received (or a 0x00 byte).
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_server.h"
#include "map_render.h"
#include "nav_state.h"
#include "ble_scan.h"

static const char *TAG = "ble_server";

#define ADV_NAME "EINK-MAP"

volatile bool ble_got_data = false;
volatile bool ble_connected = false;

/* ---- RX packet buffer ---- */
#define RX_BUF_MAX 8192
static char rx_buf[RX_BUF_MAX];
static int rx_len = 0;

/* ---- Service UUIDs (128-bit, stored LSB-first for NimBLE) ----
 * Service 5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c
 * Char    5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c
 */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x3c, 0x2b, 0x1a, 0x8e, 0x0f, 0x5c, 0x9a, 0x9f,
                     0x66, 0x4f, 0x2f, 0x2b, 0x00, 0x10, 0x7e, 0x5a);
static const ble_uuid128_t chr_uuid =
    BLE_UUID128_INIT(0x3c, 0x2b, 0x1a, 0x8e, 0x0f, 0x5c, 0x9a, 0x9f,
                     0x66, 0x4f, 0x2f, 0x2b, 0x01, 0x10, 0x7e, 0x5a);

static uint8_t own_addr_type;

/* ---- RX accumulation ---- */
static void finalize_packet(void)
{
    if (rx_len <= 0)
        return;
    rx_buf[rx_len] = '\0';
    ESP_LOGI(TAG, "packet %d bytes", rx_len);
    if (strstr(rx_buf, "<route") || strstr(rx_buf, "<nav") ||
        strstr(rx_buf, "<pos")) {
        nav_state_parse(rx_buf);
    } else {
        map_render_show(rx_buf, (size_t)rx_len);
        ble_got_data = true;
    }
    rx_len = 0;
}

static void rx_put(char b)
{
    if (rx_len < RX_BUF_MAX - 1)
        rx_buf[rx_len++] = b;
    rx_buf[rx_len] = '\0';

    if (b == 0) { /* explicit terminator */
        finalize_packet();
        return;
    }
    /* Complete when the closing tag has arrived (packets may be split). */
    if (rx_len >= 6 &&
        (strcmp(rx_buf + rx_len - 6, "</map>") == 0 ||
         strcmp(rx_buf + rx_len - 6, "</nav>") == 0))
        finalize_packet();
    else if (rx_len >= 7 && strcmp(rx_buf + rx_len - 7, "</route>") == 0)
        finalize_packet();
    else if (rx_len >= 6 && strcmp(rx_buf + rx_len - 6, "</pos>") == 0)
        finalize_packet();
}

/* ---- GATT access callback ---- */
static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
        return BLE_ATT_ERR_UNLIKELY;

    struct os_mbuf *om = ctxt->om;
    uint16_t len = OS_MBUF_PKTLEN(om);
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = 0;
        if (os_mbuf_copydata(om, i, 1, &b) != 0)
            break;
        rx_put((char)b);
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &chr_uuid.u,
                .access_cb = gatt_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 },
        },
    },
    { 0 },
};

/* ---- Advertising ---- */
static int on_gap_event(struct ble_gap_event *event, void *arg);

static void restart_adv(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)ADV_NAME;
    fields.name_len = (uint8_t)strlen(ADV_NAME);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                               &adv_params, on_gap_event, NULL);
    if (rc != 0)
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
}

static int on_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ble_connected = true;
            ESP_LOGI(TAG, "client connected");
        } else {
            ESP_LOGI(TAG, "connect failed (%d), restarting adv",
                     event->connect.status);
            restart_adv();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ble_connected = false;
        ESP_LOGI(TAG, "client disconnected (reason=%d)",
                 event->disconnect.reason);
        restart_adv();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated to %u", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        restart_adv();
        return 0;

    default:
        return 0;
    }
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer addr rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE ready, advertising as \"%s\"", ADV_NAME);
    restart_adv();
    ble_scan_start(); /* start the BLE scanner once the host is synced */
}

static void host_task(void *param)
{
    nimble_port_run(); /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ---- Public ---- */
void ble_server_init(void)
{
    ESP_LOGI(TAG, "init: heap free=%u", (unsigned)esp_get_free_heap_size());

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "init: nvs ok");

    /* Release memory reserved for Classic BT (unused on BLE-only S3), then
     * bring up the NimBLE host. NOTE: nimble_port_init() itself initializes
     * + enables the BLE controller (double-init fails on IDF 5.5), so we must
     * NOT call esp_bt_controller_init/enable here. */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "init: nimble host ok");

    ble_hs_cfg.sync_cb = on_sync;

    ble_svc_gap_device_name_set(ADV_NAME);
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "count_cfg rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "add_svcs rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "init: gatts ok");

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE server initialized");
}
