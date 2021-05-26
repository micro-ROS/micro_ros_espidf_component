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
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <control_msgs/msg/joint_jog.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
control_msgs__msg__JointJog msg;
control_msgs__msg__JointJog msg_static;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		RCSOFTCHECK(rcl_publish(&publisher, &msg_static, NULL));
	}
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
	RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
    	ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
		"esp32_publisher"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	// INIT MESSAGE MEMORY

	// --- Configuration ---

	// micro-ROS utilities allows to configure the dynamic memory initialization using a micro_ros_utilities_memory_conf_t structure
	// If some member of this struct is set to zero, the library will use the default value.
	// Check `micro_ros_utilities_memory_conf_default` in `<micro_ros_utilities/type_utilities.h>` for those defaults.

	static micro_ros_utilities_memory_conf_t conf = {0};

	// OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences

	conf.max_string_capacity = 50;
	conf.max_ros2_type_sequence_capacity = 5;
	conf.max_basic_type_sequence_capacity = 5;

	// OPTIONALLY this struct can store rules for specific members
	// !! Using the API with rules will use dynamic memory allocations for handling strings !!

	micro_ros_utilities_memory_rule_t rules[] = {
		{"header.frame_id", 30},
		{"joint_names", 3},
		{"displacements", 8}
	};
	conf.rules = rules;
	conf.n_rules = sizeof(rules) / sizeof(rules[0]);

	// --- API ---

	// Is possible to calculate the size that will be needed by a msg with a certain configuration

	size_t dynamic_size = micro_ros_utilities_get_dynamic_size(
		ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
		conf
	);

	// The total (stack, static & dynamic) memory usage of a packet will be:

	size_t message_total_size = dynamic_size + sizeof(control_msgs__msg__JointJog);

	// The message dynamic memory can be allocated using the following call.
	// This will use rcutils default allocators for getting memory.

	bool success = micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
		&msg,
		conf
	);

	// The message dynamic memory can also be allocated using a buffer.
	// This will NOT use dynamic memory for the allocation.
	// If no rules set in the conf, no dynamic allocation is guaranteed.
	// This method will use need more memory than dynamic due to alignment

	success &= micro_ros_utilities_create_static_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
		&msg_static,
		conf,
		my_buffer,
		sizeof(my_buffer)
	);

	// Dynamically allocated messages can be destroyed using:

	// success &= micro_ros_utilities_destroy_message_memory(
	//   ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
	//   &msg,
	//   conf
	// );

	// Fill the message
	msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, "framename");
	msg_static.header.frame_id = micro_ros_string_utilities_set(msg_static.header.frame_id, "framenamestatic");

	msg.velocities.size    = msg_static.velocities.size    = 3;
	msg.velocities.data[0] = msg_static.velocities.data[0] = 10.0;
	msg.velocities.data[1] = msg_static.velocities.data[1] = 11.1;
	msg.velocities.data[2] = msg_static.velocities.data[2] = 22.2;

	msg.displacements.size = 4;
	for (size_t i = 0; i < msg.displacements.size; i++)
	{
		msg.displacements.data[i] = i;
	}

	// spin executor
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
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
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
