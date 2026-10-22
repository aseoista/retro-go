#include <rg_system.h>
#include <string.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/stream_buffer.h>
#include <nvs_flash.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>

#include "transport.h"

// Bluetooth Classic Serial Port Profile, using Bluedroid in callback mode

#define CHUNK_SIZE 512

enum
{
    EV_SPP_INIT     = BIT0,
    EV_SCAN_DONE    = BIT1,
    EV_SDP_DONE     = BIT2,
    EV_OPEN         = BIT3,
    EV_CLOSED       = BIT4,
    EV_WRITE_DONE   = BIT5,
    EV_UNCONGESTED  = BIT6,
};

static EventGroupHandle_t events;
static StreamBufferHandle_t rx_buffer;
static bool initialized = false;
static uint32_t handle = 0;
static bool connected = false;
static bool write_congested = false;
static int write_status = ESP_SPP_SUCCESS;
static uint8_t sdp_channel = 0;
static char pin_code[ESP_BT_PIN_CODE_LEN + 1] = "0000";

static printer_device_t *scan_devices;
static int scan_max;
static int scan_count;

static bool parse_address(const char *str, esp_bd_addr_t out)
{
    unsigned int b[6];
    if (!str || sscanf(str, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
        return false;
    for (int i = 0; i < 6; i++)
        out[i] = b[i];
    return true;
}

static void format_address(const esp_bd_addr_t bda, char *out)
{
    sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static void scan_result(esp_bt_gap_cb_param_t *param)
{
    char address[18], name[32] = {0};
    uint32_t cod = 0;

    format_address(param->disc_res.bda, address);

    for (int i = 0; i < param->disc_res.num_prop; i++)
    {
        esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
        if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME)
        {
            snprintf(name, sizeof(name), "%.*s", prop->len, (char *)prop->val);
        }
        else if (prop->type == ESP_BT_GAP_DEV_PROP_COD)
        {
            cod = *(uint32_t *)prop->val;
        }
        else if (prop->type == ESP_BT_GAP_DEV_PROP_EIR && !name[0])
        {
            uint8_t len = 0;
            uint8_t *eir_name = esp_bt_gap_resolve_eir_data(prop->val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
            if (!eir_name)
                eir_name = esp_bt_gap_resolve_eir_data(prop->val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
            if (eir_name)
                snprintf(name, sizeof(name), "%.*s", len, (char *)eir_name);
        }
    }

    for (int i = 0; i < scan_count; i++)
    {
        if (strcmp(scan_devices[i].address, address) == 0)
        {
            if (name[0]) // We might learn the name in a later result
                strcpy(scan_devices[i].name, name);
            return;
        }
    }

    if (scan_count < scan_max)
    {
        printer_device_t *device = &scan_devices[scan_count++];
        strcpy(device->address, address);
        strcpy(device->name, name[0] ? name : address);
        device->is_printer = cod && esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_IMAGING;
        RG_LOGI("Found %s '%s' (cod=0x%06X)", address, device->name, (unsigned)cod);
    }
}

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_DISC_RES_EVT:
        if (scan_devices)
            scan_result(param);
        break;

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED)
            xEventGroupSetBits(events, EV_SCAN_DONE);
        break;

    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            RG_LOGI("Paired with '%s'", param->auth_cmpl.device_name);
        else
            RG_LOGE("Pairing failed: %d", param->auth_cmpl.stat);
        break;

    case ESP_BT_GAP_PIN_REQ_EVT: // Legacy pairing
    {
        esp_bt_pin_code_t pin = {0};
        int len = strlen(pin_code);
        memcpy(pin, pin_code, len);
        RG_LOGI("PIN requested, replying '%s'", pin_code);
        esp_bt_gap_pin_reply(param->pin_req.bda, true, len, pin);
        break;
    }

    case ESP_BT_GAP_CFM_REQ_EVT: // Secure Simple Pairing, numeric comparison
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    default:
        break;
    }
}

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        xEventGroupSetBits(events, EV_SPP_INIT);
        break;

    case ESP_SPP_DISCOVERY_COMP_EVT:
        sdp_channel = 0;
        if (param->disc_comp.status == ESP_SPP_SUCCESS)
        {
            for (int i = 0; i < param->disc_comp.scn_num; i++)
            {
                const char *service = param->disc_comp.service_name[i] ?: "";
                RG_LOGI("SDP: channel %d '%s'", param->disc_comp.scn[i], service);
                // Prefer a serial port service if there are several
                if (!sdp_channel || strstr(service, "Serial") || strstr(service, "SPP"))
                    sdp_channel = param->disc_comp.scn[i];
            }
        }
        xEventGroupSetBits(events, EV_SDP_DONE);
        break;

    case ESP_SPP_OPEN_EVT:
        if (param->open.status == ESP_SPP_SUCCESS)
        {
            handle = param->open.handle;
            connected = true;
            write_congested = false;
            xEventGroupSetBits(events, EV_UNCONGESTED);
        }
        xEventGroupSetBits(events, EV_OPEN);
        break;

    case ESP_SPP_CLOSE_EVT:
        connected = false;
        // Wake up anybody waiting on us
        xEventGroupSetBits(events, EV_CLOSED | EV_OPEN | EV_WRITE_DONE | EV_UNCONGESTED);
        break;

    case ESP_SPP_DATA_IND_EVT:
        xStreamBufferSend(rx_buffer, param->data_ind.data, param->data_ind.len, 0);
        break;

    case ESP_SPP_WRITE_EVT:
        // Callbacks are serialized, updating the congestion bit here can't race with CONG_EVT
        write_status = param->write.status;
        write_congested = param->write.cong;
        if (write_congested)
            xEventGroupClearBits(events, EV_UNCONGESTED);
        else
            xEventGroupSetBits(events, EV_UNCONGESTED);
        xEventGroupSetBits(events, EV_WRITE_DONE);
        break;

    case ESP_SPP_CONG_EVT:
        write_congested = param->cong.cong;
        if (!write_congested)
            xEventGroupSetBits(events, EV_UNCONGESTED);
        else
            xEventGroupClearBits(events, EV_UNCONGESTED);
        break;

    default:
        break;
    }
}

static bool spp_init(void)
{
    if (initialized)
        return true;

    char *pin = rg_settings_get_string(NS_APP, "PIN", "0000");
    snprintf(pin_code, sizeof(pin_code), "%s", pin);
    free(pin);

    events = xEventGroupCreate();
    rx_buffer = xStreamBufferCreate(1024, 1);

    // Bonding keys are stored in NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK)
        RG_LOGW("nvs_flash_init failed: 0x%x, pairings won't be remembered", err);

    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    esp_bt_controller_config_t config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&config)) != ESP_OK
        || (err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK
        || (err = esp_bluedroid_init()) != ESP_OK
        || (err = esp_bluedroid_enable()) != ESP_OK
        || (err = esp_bt_gap_register_callback(gap_callback)) != ESP_OK
        || (err = esp_spp_register_callback(spp_callback)) != ESP_OK
        || (err = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK)
    {
        RG_LOGE("Bluetooth init failed: 0x%x (%s)", err, esp_err_to_name(err));
        return false;
    }

    if (!(xEventGroupWaitBits(events, EV_SPP_INIT, pdTRUE, pdTRUE, pdMS_TO_TICKS(5000)) & EV_SPP_INIT))
    {
        RG_LOGE("SPP init timed out");
        return false;
    }

    esp_bt_dev_set_device_name("Retro-Go");

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE; // Just Works
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_bt_pin_code_t unused = {0};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, unused); // We answer PIN requests ourselves

    initialized = true;
    RG_LOGI("Bluetooth ready");
    return true;
}

static void spp_deinit(void)
{
    if (!initialized)
        return;
    esp_spp_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    initialized = false;
}

static int spp_scan(printer_device_t *devices, int max_devices, int timeout_ms)
{
    if (!initialized)
        return -1;

    scan_devices = devices;
    scan_max = max_devices;
    scan_count = 0;

    xEventGroupClearBits(events, EV_SCAN_DONE);
    int inquiry_length = RG_MAX(1, RG_MIN(timeout_ms / 1280, 0x30)); // In units of 1.28s
    if (esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, inquiry_length, 0) != ESP_OK)
    {
        scan_devices = NULL;
        return -1;
    }
    xEventGroupWaitBits(events, EV_SCAN_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(timeout_ms + 5000));
    esp_bt_gap_cancel_discovery();
    scan_devices = NULL;

    return scan_count;
}

static bool spp_open(const char *address)
{
    esp_bd_addr_t bda;

    if (!initialized || !parse_address(address, bda))
        return false;

    // Find the RFCOMM channel of the serial port service (it isn't always 1)
    xEventGroupClearBits(events, EV_SDP_DONE);
    esp_spp_start_discovery(bda);
    xEventGroupWaitBits(events, EV_SDP_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(15000));
    if (!sdp_channel)
    {
        RG_LOGE("No SPP service found on %s", address);
        return false;
    }

    // The printer decides whether it wants to pair, our GAP callback handles both pairing methods
    xEventGroupClearBits(events, EV_OPEN | EV_CLOSED);
    xStreamBufferReset(rx_buffer);
    esp_spp_connect(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER, sdp_channel, bda);
    xEventGroupWaitBits(events, EV_OPEN, pdTRUE, pdTRUE, pdMS_TO_TICKS(20000));

    RG_LOGI("Connection to %s channel %d: %s", address, sdp_channel, connected ? "OK" : "FAILED");
    return connected;
}

static void spp_close(void)
{
    if (!connected)
        return;
    xEventGroupClearBits(events, EV_CLOSED);
    esp_spp_disconnect(handle);
    xEventGroupWaitBits(events, EV_CLOSED, pdTRUE, pdTRUE, pdMS_TO_TICKS(3000));
    connected = false;
}

static int spp_write(const void *data, size_t length, int timeout_ms)
{
    const uint8_t *ptr = data;
    size_t written = 0;

    while (written < length)
    {
        // The printer pauses us (RFCOMM flow control) while its buffer is full
        if (!(xEventGroupWaitBits(events, EV_UNCONGESTED, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms)) & EV_UNCONGESTED))
        {
            RG_LOGE("Write timeout (congested)");
            return -1;
        }
        if (!connected)
            return -1;

        int chunk = RG_MIN(length - written, CHUNK_SIZE);
        xEventGroupClearBits(events, EV_WRITE_DONE);
        if (esp_spp_write(handle, chunk, (uint8_t *)ptr + written) != ESP_OK)
            return -1;
        if (!(xEventGroupWaitBits(events, EV_WRITE_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(timeout_ms)) & EV_WRITE_DONE)
            || !connected || write_status != ESP_SPP_SUCCESS)
        {
            RG_LOGE("Write failed (status=%d)", write_status);
            return -1;
        }
        written += chunk;
    }

    return written;
}

static int spp_read(void *buffer, size_t length, int timeout_ms)
{
    size_t received = xStreamBufferReceive(rx_buffer, buffer, length, pdMS_TO_TICKS(timeout_ms));
    if (received == 0 && !connected)
        return -1;
    return received;
}

const printer_transport_t transport_spp = {
    .name = "Bluetooth",
    .init = spp_init,
    .deinit = spp_deinit,
    .scan = spp_scan,
    .open = spp_open,
    .close = spp_close,
    .write = spp_write,
    .read = spp_read,
};
