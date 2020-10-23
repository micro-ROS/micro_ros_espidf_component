#include <uxr/client/profile/transport/serial/serial_transport_external.h>

#include <driver/uart.h>
#include <driver/gpio.h>

#define UART_TXD  (CONFIG_MICROROS_UART_TXD)
#define UART_RXD  (CONFIG_MICROROS_UART_RXD)
#define UART_RTS  (CONFIG_MICROROS_UART_RTS)
#define UART_CTS  (CONFIG_MICROROS_UART_CTS)

#define UART_BUFFER_SIZE (512)


bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{

    if (fd == 0) {
        platform->uart_port = UART_NUM_0;
    } else if (fd == 1) {
        platform->uart_port = UART_NUM_1;
    } else if (fd == 2) {
        platform->uart_port = UART_NUM_2;
    } else {
        return false;
    }

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(platform->uart_port, &uart_config) == ESP_FAIL) {
        return false;
    }
    if (uart_set_pin(platform->uart_port, UART_TXD, UART_RXD, UART_RTS, UART_CTS) == ESP_FAIL) {
        return false;
    }
    if (uart_driver_install(platform->uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) == ESP_FAIL) {
        return false;
    }

    return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{   
    return uart_driver_delete(platform->uart_port) == ESP_OK;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{
    const int txBytes = uart_write_bytes(platform->uart_port, buf, len);
    return txBytes;
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{ 
    const int rxBytes = uart_read_bytes(platform->uart_port, buf, len, timeout / portTICK_RATE_MS);
    return rxBytes;
}

