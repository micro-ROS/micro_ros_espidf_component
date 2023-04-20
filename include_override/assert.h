// Workaround for https://github.com/ros2/rosidl/pull/739

#include_next <assert.h>

#ifdef NDEBUG
#undef assert
#define assert(x) (void)0;
#endif