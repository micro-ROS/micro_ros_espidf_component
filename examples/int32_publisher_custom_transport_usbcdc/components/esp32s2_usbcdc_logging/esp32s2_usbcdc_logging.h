#ifndef ESP32S2_USBCDC_LOGGING_H
#define ESP32S2_USBCDC_LOGGING_H

#include "esp_err.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "sdkconfig.h"

#if (CONFIG_TINYUSB_CDC_COUNT < 2) 
    #warning "Define CONFIG_TINYUSB_CDC_COUNT to 2 in menuconfig if you want log over USBCDC. Otherwise, disable log output in menuconfig."
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)

#ifdef __cplusplus
extern "C" 
{
#endif

esp_err_t esp32s2_usbcdc_logging_init(void);
esp_err_t esp32s2_usbcdc_logging_deinit(void);

#ifdef __cplusplus
}
#endif

#else
#error "Logging over USB-CDC is only supported on ESP32-S2 or ESP32-S3 targets"
#endif

#endif // ESP32S2_USBCDC_LOGGING_H
