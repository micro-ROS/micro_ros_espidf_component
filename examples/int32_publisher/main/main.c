#include <string.h>

#include "uxr/client/config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"

#include <driver/uart.h>
#include <driver/gpio.h>

#include <uros_network_interfaces.h>

extern void appMain(void *argument);

void app_main(void)
{   
#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

    //start microROS task
    xTaskCreate(appMain, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, 5, NULL);
}
