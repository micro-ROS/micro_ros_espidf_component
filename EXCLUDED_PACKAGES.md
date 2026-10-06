# Excluded ROS 2 Packages

After cloning, [`libmicroros.mk`](./libmicroros.mk) removes the packages below from the workspace before building [`libmicroros.a`](./libmicroros.a).

[`tracetools`](https://github.com/ros2/ros2_tracing/tree/rolling/tracetools) is kept because `rcl` depends on it at build time (tracing is disabled via [`colcon.meta`](./colcon.meta)).

## Can't Build Yet (Need Upstream Patches)

- [`rosidl_buffer`](https://github.com/ros2/rosidl/tree/rolling/rosidl_buffer)
- [`rosidl_buffer_backend`](https://github.com/ros2/rosidl/tree/rolling/rosidl_buffer_backend)
- [`rosidl_buffer_backend_registry`](https://github.com/ros2/rosidl/tree/rolling/rosidl_buffer_backend_registry)

[`rosidl`](https://github.com/ros2/rosidl) is pinned to an older commit until [`ros2/rosidl#942`](https://github.com/ros2/rosidl/pull/942) can be built for microcontrollers.

## Not Useful for Micro-ROS

- [`common_interfaces`](https://github.com/ros2/common_interfaces/tree/rolling/common_interfaces)
- [`lttngpy`](https://github.com/ros2/ros2_tracing/tree/rolling/lttngpy)
- [`rcl_lifecycle`](https://github.com/micro-ROS/rcl/tree/upstream-patches/rcl_lifecycle)
- [`rcl_yaml_param_parser`](https://github.com/micro-ROS/rcl/tree/upstream-patches/rcl_yaml_param_parser)
- [`rcl_logging_implementation`](https://github.com/ros2/rcl_logging/tree/rolling/rcl_logging_implementation)
- [`rcl_logging_spdlog`](https://github.com/ros2/rcl_logging/tree/rolling/rcl_logging_spdlog)
- [`rclc_examples`](https://github.com/ros2/rclc/tree/rolling/rclc_examples)
- [`rclc_lifecycle`](https://github.com/ros2/rclc/tree/rolling/rclc_lifecycle)
- [`rmw_security_common`](https://github.com/ros2/rmw/tree/rolling/rmw_security_common)
- [`rmw_test_fixture`](https://github.com/ros2/ament_cmake_ros/tree/rolling/rmw_test_fixture)
- [`rmw_test_fixture_implementation`](https://github.com/ros2/ament_cmake_ros/tree/rolling/rmw_test_fixture_implementation)
- [`ros2trace`](https://github.com/ros2/ros2_tracing/tree/rolling/ros2trace)
- [`rosidl_generator_tests`](https://github.com/ros2/rosidl/tree/rolling/rosidl_generator_tests)
- [`rosidl_typesupport_introspection_cpp`](https://github.com/ros2/rosidl/tree/rolling/rosidl_typesupport_introspection_cpp)
- [`rosidl_typesupport_introspection_tests`](https://github.com/ros2/rosidl/tree/rolling/rosidl_typesupport_introspection_tests)
- [`rosidl_typesupport_microxrcedds/test`](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds/tree/rolling/test)
- [`rosidl_typesupport_tests`](https://github.com/micro-ROS/rosidl_typesupport/tree/rolling/rosidl_typesupport_tests)
- [`sensor_msgs_py`](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs_py)
- [`test_rmw_implementation`](https://github.com/ros2/rmw_implementation/tree/rolling/test_rmw_implementation)
- [`test_ros2trace`](https://github.com/ros2/ros2_tracing/tree/rolling/test_ros2trace)
- [`test_tracetools`](https://github.com/ros2/ros2_tracing/tree/rolling/test_tracetools)
- [`test_tracetools_launch`](https://github.com/ros2/ros2_tracing/tree/rolling/test_tracetools_launch)
- [`tracetools_launch`](https://github.com/ros2/ros2_tracing/tree/rolling/tracetools_launch)
- [`tracetools_read`](https://github.com/ros2/ros2_tracing/tree/rolling/tracetools_read)
- [`tracetools_test`](https://github.com/ros2/ros2_tracing/tree/rolling/tracetools_test)
- [`tracetools_trace`](https://github.com/ros2/ros2_tracing/tree/rolling/tracetools_trace)
