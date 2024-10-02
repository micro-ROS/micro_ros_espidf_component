#include "esp32s2_usbcdc_transport.h"

// Open USB-CDC
bool esp32s2_usbcdc_open(struct uxrCustomTransport* transport) {
    const tinyusb_config_t tinyusb_config = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    esp_err_t ret = tinyusb_driver_install(&tinyusb_config);

    if (ret == ESP_ERR_INVALID_ARG || ret == ESP_FAIL) {
        return ret;
    }

    tinyusb_cdcacm_itf_t* cdc_port = (tinyusb_cdcacm_itf_t*)transport->args;

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = *cdc_port,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    if (tusb_cdc_acm_init(&acm_cfg) != ESP_OK) {
        return false;
    }

    return true;
}

// Close USB-CDC
bool esp32s2_usbcdc_close(struct uxrCustomTransport* transport) {
    tinyusb_cdcacm_itf_t* cdc_port = (tinyusb_cdcacm_itf_t*)transport->args;
    return (tusb_cdc_acm_deinit(*cdc_port) == ESP_OK) ? true : false;
}

// Write to USB-CDC
size_t esp32s2_usbcdc_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err) {
    tinyusb_cdcacm_itf_t* cdc_port = (tinyusb_cdcacm_itf_t*)transport->args;
    size_t tx_size = tinyusb_cdcacm_write_queue(*cdc_port, buf, len);
    tinyusb_cdcacm_write_flush(*cdc_port, 0);
    return tx_size;
}

// Read from USB-CDC
size_t esp32s2_usbcdc_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    tinyusb_cdcacm_itf_t* cdc_port = (tinyusb_cdcacm_itf_t*)transport->args;
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(*cdc_port, buf, len, &rx_size);
    return (ret == ESP_OK) ? rx_size : 0;
}