
#ifndef _UXR_CLIENT_SERIAL_TRANSPORT_ESP32_H_
#define _UXR_CLIENT_SERIAL_TRANSPORT_ESP32_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <driver/uart.h>

typedef struct uxrSerialPlatform
{
    uart_port_t uart_port;
} uxrSerialPlatform;

#ifdef __cplusplus
}
#endif

#endif //_UXR_CLIENT_SERIAL_TRANSPORT_ESP32_H_

