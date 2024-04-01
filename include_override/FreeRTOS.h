// Just for micro-ROS compatibility remove ESP_STATIC_ASSERT from micro-ROS build
#undef ESP_STATIC_ASSERT
#define ESP_STATIC_ASSERT(...) ((void)0);

// Include proper path to FreeRTOS header file
#include "freertos/FreeRTOS.h"
