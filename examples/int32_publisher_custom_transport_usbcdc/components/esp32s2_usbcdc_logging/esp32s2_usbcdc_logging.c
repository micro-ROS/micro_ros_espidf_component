#include "esp32s2_usbcdc_logging.h"

// Initialize USB-CDC logging
esp_err_t esp32s2_usbcdc_logging_init(void)
{
    const tinyusb_config_t tinyusb_config = {
        .descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    esp_err_t ret = tinyusb_driver_install(&tinyusb_config);

    if (ret == ESP_ERR_INVALID_ARG || ret == ESP_FAIL) {
        return ret;
    }

    tinyusb_config_cdcacm_t acm_config = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_1,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };

    ret = tusb_cdc_acm_init(&acm_config);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_tusb_init_console(TINYUSB_CDC_ACM_1);

    return ret;
}

// Deinitialize USB-CDC logging
esp_err_t esp32s2_usbcdc_logging_deinit(void)
{
    esp_err_t ret = esp_tusb_deinit_console(TINYUSB_CDC_ACM_1);

    return ret;
}
