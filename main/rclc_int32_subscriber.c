#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <profiling_message/msg/profiling_message.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <malloc.h>

#include <rmw_uros/options.h>


#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);goto clean;;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ---------------------------
//    Profiling functions
// ---------------------------

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

static inline size_t block_data_size(const heap_block_t *block)
{
    intptr_t next = (intptr_t)block->header & NEXT_BLOCK_MASK;
    intptr_t this = (intptr_t)block;
    if (next == 0) {
        return 0; /* this is the last block in the heap */
    }
    return next - this - sizeof(block->header);
}

static inline heap_block_t *get_block(const void *data_ptr)
{
	if(data_ptr == NULL)
		return 0;
    return (heap_block_t *)((char *)data_ptr - offsetof(heap_block_t, data));
}



void * __freertos_allocate(size_t size, void * state){
	(void) state;

	absoluteUsedMemory += size;
	usedMemory += size;
	return malloc(size);
}

void __freertos_deallocate(void * pointer, void * state){
  (void) state;
  
  if (NULL != pointer){
	    heap_block_t *pb = get_block(pointer);
		size_t size = block_data_size(pb);
		usedMemory -= size;
		free(pointer);
  }
}

void * __freertos_reallocate(void * pointer, size_t size, void * state){
  	(void) state;
	
	if (NULL != pointer){
		heap_block_t *pb = get_block(pointer);
		size_t old_size = block_data_size(pb);
		absoluteUsedMemory += size;
		usedMemory -= old_size;
		usedMemory += size;
		return realloc(pointer, size);
	}else{
		absoluteUsedMemory += size;
		usedMemory += size;
		return malloc(size);
	}
}

void * __freertos_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state){
  	(void) state;

	absoluteUsedMemory += number_of_elements*size_of_element;
	usedMemory += number_of_elements*size_of_element;
  	return calloc(number_of_elements, size_of_element);
}

// ---------------------------
//   End Profiling functions
// ---------------------------

uint32_t n_entities;
uint32_t topic_size;
rcl_subscription_t * subscribers;
profiling_message__msg__ProfilingMessage msg;
uint8_t counter = 0;
uint8_t created = 0;

void subscription_callback(const void * msgin)
{
	const profiling_message__msg__ProfilingMessage * msg = (const profiling_message__msg__ProfilingMessage *)msgin;
	if (msg->payload.size == topic_size)
	{
		// printf("received %d\n", msg->payload.size);
		counter++;
	}
}

void appMain(void * arg)
{
	// Init
	uint32_t * args = (uint32_t*) arg;
	n_entities = args[0];
    topic_size = args[1];
	absoluteUsedMemory = 0;
	usedMemory = 0;
	counter = 0;
	created = 0;

	printf("lauching %u entities of %u B available mem:%d \n", args[0], args[1], xPortGetFreeHeapSize());

	subscribers = (rcl_subscription_t*)calloc(n_entities,sizeof(rcl_subscription_t));
	memset(subscribers, 0, n_entities*sizeof(rcl_subscription_t));

	msg.payload.data = (uint8_t*)calloc(topic_size, sizeof(uint8_t));
	memset(msg.payload.data, 'a', topic_size);
	msg.payload.size = topic_size;
	msg.payload.capacity = topic_size;

	if(subscribers == NULL || msg.payload.data == NULL){
		printf("INIT ERROR\n");
		vTaskDelete(NULL);
		goto clean;
	}

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = __freertos_allocate;
    freeRTOS_allocator.deallocate = __freertos_deallocate;
    freeRTOS_allocator.reallocate = __freertos_reallocate;
    freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n",__LINE__); 
		vTaskDelete(NULL);
		goto clean;
    }

	// micro-ROS
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	memset(&support, 0, sizeof(rclc_support_t));
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	rmw_uros_options_set_client_key(0xDEADBEEF, rmw_options);

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "freertos_int32_subscriber", "", &support));

	// create subscriber
	for(uint8_t i = 0; i < n_entities; i++){
		rcl_ret_t ret = rclc_subscription_init_default(
			&subscribers[i],
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(profiling_message, msg, ProfilingMessage),
			"freertos_int32_publisher");
		if(ret == RCL_RET_OK){
			created++;
		}else{
			goto clean;
		}
		usleep(100000);
	}

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, n_entities, &allocator));

	unsigned int rcl_wait_timeout = 100;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	for(uint8_t i = 0; i < n_entities; i++){
		RCCHECK(rclc_executor_add_subscription(&executor, &subscribers[i], &msg, &subscription_callback, ON_NEW_DATA));
	}

	while(counter < 10*n_entities){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500));
	}

	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	printf("%d,%d,%d,%d,%d\n",
			n_entities,
			topic_size,
			(12*2048) - uxHighWaterMark,
			absoluteUsedMemory,
			usedMemory);

	rclc_executor_fini(&executor);

clean:

	for(uint8_t i = 0; i < created; i++){
		RCSOFTCHECK(rcl_subscription_fini(&subscribers[i], &node));
	}
	rcl_node_fini(&node);
	rcl_shutdown(&support.context);
	rcl_context_fini(&support.context);
	rclc_support_fini(&support);
	
	free(msg.payload.data);
	free(subscribers);

  	vTaskDelete(NULL);
	for(;;){};
}
