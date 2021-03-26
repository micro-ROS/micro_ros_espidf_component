#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_uros/options.h>
#include "uxr/client/config.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void publisher_task(void * arg)
{
	rcl_publisher_t * publisher = (rcl_publisher_t *) arg;
	std_msgs__msg__Int32 msg;
	msg.data = (int32_t) arg;

	while (1)
	{
		RCSOFTCHECK(rcl_publish(publisher, &msg, NULL));
		msg.data++;
		sleep(1);
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received: %d\n", msg->data);
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_multithread", "", &support));

	// create publisher
	rcl_publisher_t publisher_1;
	RCCHECK(rclc_publisher_init_default(
		&publisher_1,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_multithread_publisher_1"));
	
	rcl_publisher_t publisher_2;
	RCCHECK(rclc_publisher_init_default(
		&publisher_2,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_multithread_publisher_2"));

	rcl_subscription_t subscriber_1;
		RCCHECK(rclc_subscription_init_default(
		&subscriber_1,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_multithread_publisher_1"));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

	std_msgs__msg__Int32 msg;
	rclc_executor_add_subscription(&executor, &subscriber_1, &msg, subscription_callback, ON_NEW_DATA);

	msg.data = 0;

	xTaskCreate(publisher_task, 
		"publisher_1_task", 
		4000, 
		(void*) &publisher_1,
		CONFIG_MICRO_ROS_APP_TASK_PRIO+1, 
		NULL); 
		
	xTaskCreate(publisher_task, 
		"publisher_2_task", 
		4000, 
		(void*) &publisher_2,
		CONFIG_MICRO_ROS_APP_TASK_PRIO+1, 
		NULL);

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher_1, &node));
	RCCHECK(rcl_publisher_fini(&publisher_2, &node));
	RCCHECK(rcl_subscription_fini(&subscriber_1, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{   
#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task, 
            "uros_task", 
            20000, 
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO, 
            NULL); 
}