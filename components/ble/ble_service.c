#include "ble_service.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_random.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "ble_service";

#define PROFILE_APP_ID   0
#define DEVICE_NAME      "AnklePump"
#define SVC_NUM_HANDLES  4   /* svc decl + char decl + char value + CCCD */

/* Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b (wire-format bytes) */
static esp_bt_uuid_t svc_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {
        0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
        0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f
    }
};

/* Characteristic UUID: beb5483e-36e1-4688-b7f5-ea07361b26a8 (wire-format bytes) */
static esp_bt_uuid_t chr_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = {
        0xa8, 0x26, 0x1b, 0x73, 0xea, 0x07, 0xf5, 0xb7,
        0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
    }
};

/* Global state */
static esp_gatt_if_t gatts_if       = ESP_GATT_IF_NONE;
static uint16_t      conn_id        = 0xFFFF;
static uint16_t      svc_handle     = 0;
static uint16_t      chr_handle     = 0;
static uint16_t      cccd_handle    = 0;
static bool          is_connected   = false;
static bool          notify_enabled = false;

/* Forward declarations */
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param);

/* ── Advertising helpers ───────────────────────────────────────────── */

static void start_advertising(void)
{
    esp_ble_adv_params_t adv_params = {
        .adv_int_min       = 0x20,
        .adv_int_max       = 0x40,
        .adv_type          = ADV_TYPE_IND,
        .own_addr_type     = BLE_ADDR_TYPE_RANDOM,
        .channel_map       = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    esp_ble_gap_start_advertising(&adv_params);
}

static void set_adv_data(void)
{
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp        = false,
        .include_name        = true,
        .include_txpower     = false,
        .min_interval        = 0x20,
        .max_interval        = 0x40,
        .appearance          = 0,
        .manufacturer_len    = 0,
        .p_manufacturer_data = NULL,
        .service_data_len    = 0,
        .p_service_data      = NULL,
        .service_uuid_len    = ESP_UUID_LEN_128,
        .p_service_uuid      = svc_uuid.uuid.uuid128,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    esp_ble_gap_config_adv_data(&adv_data);
}

/* ── GATT event handler ─────────────────────────────────────────────── */

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t evt_gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            gatts_if = evt_gatts_if;
            ESP_LOGI(TAG, "REG_EVT ok, creating service");
            esp_gatt_srvc_id_t srvc_id;
            srvc_id.is_primary = true;
            srvc_id.id.inst_id = 0;
            srvc_id.id.uuid = svc_uuid;
            esp_ble_gatts_create_service(gatts_if, &srvc_id, SVC_NUM_HANDLES);
        } else {
            ESP_LOGE(TAG, "REG_EVT failed: %d", param->reg.status);
        }
        break;

    case ESP_GATTS_CREATE_EVT:
        if (param->create.status == ESP_GATT_OK) {
            svc_handle = param->create.service_handle;
            ESP_LOGI(TAG, "CREATE_EVT ok, handle=%d", svc_handle);

            esp_attr_value_t char_val = {
                .attr_max_len = 600,
                .attr_len     = 0,
                .attr_value   = NULL,
            };
            esp_ble_gatts_add_char(svc_handle,
                &chr_uuid,
                ESP_GATT_PERM_READ,
                ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                &char_val, NULL);

            esp_ble_gatts_start_service(svc_handle);
        } else {
            ESP_LOGE(TAG, "CREATE_EVT failed: %d", param->create.status);
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status == ESP_GATT_OK) {
            chr_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "ADD_CHAR_EVT ok, handle=%d", chr_handle);

            esp_bt_uuid_t cccd_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
            };
            uint16_t cccd_init = 0x0000;
            esp_attr_value_t cccd_val = {
                .attr_max_len = 2,
                .attr_len     = 2,
                .attr_value   = (uint8_t *)&cccd_init,
            };
            esp_ble_gatts_add_char_descr(svc_handle,
                &cccd_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                &cccd_val, NULL);
        } else {
            ESP_LOGE(TAG, "ADD_CHAR_EVT failed: %d", param->add_char.status);
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.status == ESP_GATT_OK) {
            cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCCD descriptor added, handle=%d", cccd_handle);
        }
        break;

    case ESP_GATTS_START_EVT:
        if (param->start.status == ESP_GATT_OK) {
            ESP_LOGI(TAG, "START_EVT ok, setting adv data");
            set_adv_data();
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        conn_id      = param->connect.conn_id;
        is_connected = true;
        ESP_LOGI(TAG, "Connected, conn_id=%d", conn_id);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Disconnected, reason=0x%x", param->disconnect.reason);
        is_connected   = false;
        notify_enabled = false;
        conn_id        = 0xFFFF;
        set_adv_data();
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU updated: %d", param->mtu.mtu);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == cccd_handle && param->write.len == 2) {
            uint16_t val = param->write.value[0]
                         | (param->write.value[1] << 8);
            notify_enabled = ((val & 0x0001) != 0);
            ESP_LOGI(TAG, "CCCD written: 0x%04x, notify %s",
                     val, notify_enabled ? "enabled" : "disabled");
        }
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(evt_gatts_if,
                param->write.conn_id, param->write.trans_id,
                ESP_GATT_OK, NULL);
        }
        break;

    case ESP_GATTS_READ_EVT:
        /* Bluedroid returns the current attribute value automatically */
        break;

    default:
        break;
    }
}

/* ── GAP event handler ──────────────────────────────────────────────── */

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        if (param->adv_data_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            start_advertising();
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising as '%s'", DEVICE_NAME);
        }
        break;

    default:
        break;
    }
}

/* ── Public API ─────────────────────────────────────────────────────── */

void ble_service_init(void)
{
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.sleep_mode = ESP_BT_SLEEP_MODE_NONE;  /* prevent missing conn events */
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bt_controller_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bt_controller_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    /* ESP32-S3 has no public BT MAC — must set a static random address.
     * Persist address in NVS so the phone sees the same device each boot. */
    uint8_t rand_addr[6];
    nvs_handle_t nvs_h;
    bool addr_from_nvs = false;

    if (nvs_open("ble_config", NVS_READWRITE, &nvs_h) == ESP_OK) {
        size_t len = sizeof(rand_addr);
        if (nvs_get_blob(nvs_h, "rand_addr", rand_addr, &len) == ESP_OK && len == 6) {
            addr_from_nvs = true;
        } else {
            esp_fill_random(rand_addr, 6);
            rand_addr[0] |= 0xC0;  /* top two bits = 11 → static random address */
            nvs_set_blob(nvs_h, "rand_addr", rand_addr, 6);
            nvs_commit(nvs_h);
        }
        nvs_close(nvs_h);
    } else {
        esp_fill_random(rand_addr, 6);
        rand_addr[0] |= 0xC0;
    }

    ret = esp_ble_gap_set_rand_addr(rand_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_rand_addr failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "%s addr: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_from_nvs ? "Saved" : "New random",
             rand_addr[5], rand_addr[4], rand_addr[3],
             rand_addr[2], rand_addr[1], rand_addr[0]);

    /* Disable pairing & bonding — no auth needed for notify-only service */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;
    esp_ble_io_cap_t   iocap    = ESP_IO_CAP_NONE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gatts_register_callback failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gap_register_callback failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_set_device_name(DEVICE_NAME);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gap_set_device_name failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gatts_app_register failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Init complete, starting GATT registration...");
}

void ble_notify_movement(const char *json_str, size_t len)
{
    if (!is_connected || !notify_enabled) {
        return;
    }

    esp_err_t ret = esp_ble_gatts_send_indicate(
        gatts_if, conn_id, chr_handle,
        len, (uint8_t *)json_str, false);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "send_indicate failed: %s (len=%d)",
                 esp_err_to_name(ret), (int)len);
    }
}
