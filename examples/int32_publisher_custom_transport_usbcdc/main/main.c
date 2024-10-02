#include <stdio.h>
#include <unistd.h>
 
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp32s2_usbcdc_logging.h"
#include "esp32s2_usbcdc_transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/int32.h>

#define DOMAIN_ID 100
#define TIMER_PERIOD 1000

#define NODE_NAME "esp32s2"
#define PUBLISHER_NAME "/int32_publisher_usbcdc"
#define TOPIC_NAME NODE_NAME PUBLISHER_NAME

#define NUMBER_OF_HANDLES 1

#define TAG_CALLBACK "TIMER_CALLBACK"
#define TAG_MAIN "MAIN"
#define TAG_TASK "MICRO_ROS"
 
// Check for error in micro-ROS initialization, aborting the process
#define RCCHECK(fn) {                                                         					 		\
    rcl_ret_t temp_rc = fn;                                                    					 		\
    if ((temp_rc != RCL_RET_OK)) {                                            					 		\
      	ESP_LOGE("RCCHECK", "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);		\
      	vTaskDelete(NULL);                                                    					 		\
    }                                                                         					 		\
}

// Check for error in micro-ROS initialization, continuing the process
#define RCSOFTCHECK(fn) {                                                                              	\
    rcl_ret_t temp_rc = fn;                                                                            	\
    if ((temp_rc != RCL_RET_OK)) {                                            						   	\
      	ESP_LOGW("RCSOFTCHECK","Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); 	\
    }                                                                                                  	\
}

rcl_publisher_t publisher; // Publisher
std_msgs__msg__Int32 msg; // Message to be published

// Timer callback. Publishes a message
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		// Publish message to topic
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		ESP_LOGI(TAG_CALLBACK, "Message published: %ld", msg.data++);
	}
}
 
void micro_ros_task(void *arg) {

	rcl_allocator_t allocator = rcl_get_default_allocator(); // Initialize allocator
	rclc_support_t support = {0}; // Initialize support
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options(); // Initialize init_options
	
	// Initialize init_options
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	// Set domain id
	RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));
	
	// Initialize support with options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	
	// Create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	rcl_node_options_t node_ops = rcl_node_get_default_options();
	RCCHECK(rclc_node_init_with_options(&node, NODE_NAME, "", &support, &node_ops));
	
	// Create publisher
	RCCHECK(rclc_publisher_init_default(
			&publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			TOPIC_NAME));
	
	// Create timer
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default2(
			&timer, 
			&support, 
			RCL_MS_TO_NS(TIMER_PERIOD), 
			timer_callback, 
			true));
	
	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, NUMBER_OF_HANDLES, &allocator));

	// Add timer to executor
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	
	msg.data = 0; // Initialize message data
	rcl_ret_t spin_ret; // Return code for spin

	while (true) {
		spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		if (spin_ret != RCL_RET_OK) {
			ESP_LOGE(TAG_TASK, "rclc_executor_spin_some() failed");
		}
		usleep(10000);
	}
	
	// Free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
	ESP_LOGE(TAG_TASK, "Task finished");
	
	vTaskDelete(NULL);
}
 
static tinyusb_cdcacm_itf_t cdc_port = TINYUSB_CDC_ACM_0; // interface USB-CDC
 
void app_main(void) {
 
// Initialize logging over USB-CDC
#if (CONFIG_TINYUSB_CDC_COUNT >= 2)
	if (esp32s2_usbcdc_logging_init() == ESP_OK) {
		ESP_LOGI(TAG_MAIN, "USB-CDC Logging initialized");
	}
#endif

// Set micro-ROS custom transport layer
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
	rmw_ret_t ret = rmw_uros_set_custom_transport(
					true, 
					(void *)&cdc_port, 
					esp32s2_usbcdc_open, 
					esp32s2_usbcdc_close,
					esp32s2_usbcdc_write, 
					esp32s2_usbcdc_read);
	
	if (ret != RMW_RET_OK) {
		ESP_LOGE(TAG_MAIN, "Fail to set micro-ROS custom transport layer");
		return;
	}
#else
#error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

	// Start micro-ROS task
	TaskHandle_t task_handle = NULL;	
	xTaskCreate(micro_ros_task, 
				"uros_task", 
				CONFIG_MICRO_ROS_APP_STACK, 
				NULL,
				CONFIG_MICRO_ROS_APP_TASK_PRIO, 
				&task_handle);
	
	if (task_handle != NULL) {
		ESP_LOGI(TAG_MAIN, "micro-ROS task created");
	}
}