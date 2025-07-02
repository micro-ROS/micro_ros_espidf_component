#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <uros_network_interfaces.h>
#include <uxr/client/config.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rclc_parameter_server_t param_server;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;

    int64_t value;
    rclc_parameter_get_int(&param_server, "param2", &value);
    value++;
    rclc_parameter_set_int(&param_server, "param2", value);
}

bool on_parameter_changed(const Parameter *old_param, const Parameter *new_param, void *context)
    {
    printf("Parameter %s modified.", new_param->name.data);
    switch (new_param->value.type)
    {
    case RCLC_PARAMETER_BOOL:
        printf(" New value: %d (bool)", new_param->value.bool_value);
        break;
    case RCLC_PARAMETER_INT:
        printf(" New value: %lld (int)", new_param->value.integer_value);
        break;
    case RCLC_PARAMETER_DOUBLE:
        printf(" New value: %f (double)", new_param->value.double_value);
        break;
    default:
        break;
    }
    printf("\n");
    return true;
}

void micro_ros_task(void * arg)
{

 	rcl_ret_t rc;
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    rcl_node_t node;
    rclc_node_init_default(&node, "esp32_param_node", "", &support);

    // Create parameter service
    rclc_parameter_server_init_default(&param_server, &node);

    // create timer,
    rcl_timer_t timer;
    rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true);

    // Create executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1, &allocator);
    rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
    rclc_executor_add_timer(&executor, &timer);

    // Add parameters
    rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "param2", RCLC_PARAMETER_INT);
    rclc_add_parameter(&param_server, "param3", RCLC_PARAMETER_DOUBLE);

    rclc_parameter_set_bool(&param_server, "param1", false);
    rclc_parameter_set_int(&param_server, "param2", 10);
    rclc_parameter_set_double(&param_server, "param3", 0.01);

    bool param1;
    int64_t param2;
    double param3;

    rclc_parameter_get_bool(&param_server, "param1", &param1);
    rclc_parameter_get_int(&param_server, "param2", &param2);
    rclc_parameter_get_double(&param_server, "param3", &param3);

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

    // clean up
    rc = rclc_executor_fini(&executor);
    rc += rclc_parameter_server_fini(&param_server, &node);
    rc += rcl_node_fini(&node);

    if (rc != RCL_RET_OK) {
        printf("Error while cleaning up!\n");
    }

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