#ifndef ESP32S2_USBCDC_TRANSPORT_H
#define ESP32S2_USBCDC_TRANSPORT_H

#include <uxr/client/transport.h>

#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)

#ifdef __cplusplus
extern "C" 
{
#endif

bool esp32s2_usbcdc_open(struct uxrCustomTransport* transport);
bool esp32s2_usbcdc_close(struct uxrCustomTransport* transport);
size_t esp32s2_usbcdc_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err);
size_t esp32s2_usbcdc_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#else
#error "This transport is only supported on ESP32-S2 or ESP32-S3 targets"
#endif // CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3

#endif // ESP32S2_USBCDC_TRANSPORT_H