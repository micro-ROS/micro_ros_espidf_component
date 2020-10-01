#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include <allocators.h>
#endif

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include "example_interfaces/srv/add_two_ints.h"

#include <rmw_uros/options.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);  vTaskDelete(NULL);;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define SERVICES_NUMBER 1

/**
 * @brief Functions for memory profiling, including custom allocators
 */
UBaseType_t uxHighWaterMark;
static int absoluteUsedMemory = 0;
static int usedMemory = 0;

typedef struct heap_block {
  intptr_t header;
  union {
    uint8_t data[1];
    struct heap_block *next_free;
  };
} heap_block_t;

#define NEXT_BLOCK_MASK (~3) /* AND header with this mask to get pointer to next block (free or used) */

static inline size_t block_data_size(const heap_block_t *block) {
  intptr_t next = (intptr_t)block->header & NEXT_BLOCK_MASK;
  intptr_t this = (intptr_t)block;
  if (next == 0) {
    return 0; /* this is the last block in the heap */
  }
  return next - this - sizeof(block->header);
}

static inline heap_block_t *get_block(const void *data_ptr) {
	if(data_ptr == NULL) {
    return 0;
  }
  return (heap_block_t *)((char *)data_ptr - offsetof(heap_block_t, data));
}



void * __freertos_allocate(size_t size, void * state) {
	(void) state;

	absoluteUsedMemory += size;
	usedMemory += size;
	return malloc(size);
}

void __freertos_deallocate(void * pointer, void * state) {
  (void) state;

  if (NULL != pointer){
	    heap_block_t *pb = get_block(pointer);
		size_t size = block_data_size(pb);
		usedMemory -= size;
		free(pointer);
  }
}

void * __freertos_reallocate(void * pointer, size_t size, void * state) {
  	(void) state;

	if (NULL != pointer) {
		heap_block_t *pb = get_block(pointer);
		size_t old_size = block_data_size(pb);
		absoluteUsedMemory += size;
		usedMemory -= old_size;
		usedMemory += size;
		return realloc(pointer, size);
	} else {
		absoluteUsedMemory += size;
		usedMemory += size;
		return malloc(size);
	}
}

void * __freertos_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state) {
  	(void) state;

	absoluteUsedMemory += number_of_elements*size_of_element;
	usedMemory += number_of_elements*size_of_element;
  	return calloc(number_of_elements, size_of_element);
}

// TODO(jamoralp): update using RCLC convenience functions once services are implemented there.
void appMain(void *argument)
{
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = __freertos_allocate;
  freeRTOS_allocator.deallocate = __freertos_deallocate;
  freeRTOS_allocator.reallocate = __freertos_reallocate;
  freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

  rcl_init_options_t options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&options, freeRTOS_allocator));

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(0, NULL, &options, &context));

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node, "addtwoints_server_freertos", "", &context, &node_ops));

  rcl_service_t services[SERVICES_NUMBER];
  rcl_service_options_t service_op = rcl_service_get_default_options();
  service_op.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  const rosidl_service_type_support_t * service_type_support =
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  for (size_t i = 0; i < SERVICES_NUMBER; ++i) {
    services[i] = rcl_get_zero_initialized_service();
    char service_name[30];
    sprintf(service_name, "addtwoints_freertos_%d", (int)i);
    RCCHECK(rcl_service_init(&services[i], &node, service_type_support, service_name, &service_op));
  }

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, 0, 0, 0, 0, 1, 0, &context, freeRTOS_allocator));

  uint8_t number_of_assisted_requests = 0;
  do {
    for (size_t i = 0; i < SERVICES_NUMBER; ++i) {
      RCSOFTCHECK(rcl_wait_set_clear(&wait_set));
      size_t index_service;
      RCSOFTCHECK(rcl_wait_set_add_service(&wait_set, &services[i], &index_service));

      rcl_wait(&wait_set, RCL_MS_TO_NS(100));

      for (size_t i = 0; i < wait_set.size_of_services; ++i) {
        if (wait_set.services[i]) {
          rmw_request_id_t req_id;
          example_interfaces__srv__AddTwoInts_Request req;
          example_interfaces__srv__AddTwoInts_Request__init(&req);
          RCSOFTCHECK(rcl_take_request(&services[i], &req_id, &req));

          printf("Service request value: %d + %d. Seq %d\n", (int)req.a, (int)req.b, (int) req_id.sequence_number);

          example_interfaces__srv__AddTwoInts_Response res;
          example_interfaces__srv__AddTwoInts_Response__init(&res);

          res.sum = req.a + req.b;

          RCSOFTCHECK(rcl_send_response(&services[i], &req_id, &res));
          number_of_assisted_requests++;
        }
      }
    }
    usleep(10000);

  } while (number_of_assisted_requests < (SERVICES_NUMBER * 10));

  for (size_t i = 0; i < SERVICES_NUMBER; ++i) {
    RCCHECK(rcl_service_fini(&services[i], &node));
  }
  RCCHECK(rcl_node_fini(&node));

  printf("Dynamic total: %d\n",absoluteUsedMemory);
  printf("Dynamic transactional: %d\n",usedMemory);

  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  printf("Stack peak: %d\n", (12*2048) - uxHighWaterMark);

  vTaskDelete(NULL);
}