EXTENSIONS_DIR = $(shell pwd)
UROS_DIR = $(EXTENSIONS_DIR)/micro_ros_src
BUILD_DIR ?= $(EXTENSIONS_DIR)/build

DEBUG ?= 0

ifeq ($(DEBUG), 1)
	BUILD_TYPE = Debug
else
	BUILD_TYPE = Release
endif

all: $(EXTENSIONS_DIR)/libmicroros.a

clean:
	rm -rf $(EXTENSIONS_DIR)/libmicroros.a; \
	rm -rf $(EXTENSIONS_DIR)/include; \
	rm -rf $(EXTENSIONS_DIR)/esp32_toolchain.cmake; \
	rm -rf $(EXTENSIONS_DIR)/micro_ros_dev; \
	rm -rf $(EXTENSIONS_DIR)/micro_ros_src;

$(EXTENSIONS_DIR)/esp32_toolchain.cmake: $(EXTENSIONS_DIR)/esp32_toolchain.cmake.in
	rm -f $(EXTENSIONS_DIR)/esp32_toolchain.cmake; \
	cat $(EXTENSIONS_DIR)/esp32_toolchain.cmake.in | \
		sed "s/@CMAKE_C_COMPILER@/$(subst /,\/,$(X_CC))/g" | \
		sed "s/@CMAKE_CXX_COMPILER@/$(subst /,\/,$(X_CXX))/g" | \
		sed "s/@IDF_TARGET@/$(subst /,\/,$(IDF_TARGET))/g" | \
		sed "s/@IDF_PATH@/$(subst /,\/,$(IDF_PATH))/g" | \
		sed "s/@BUILD_CONFIG_DIR@/$(subst /,\/,$(BUILD_DIR)/config)/g" \
		> $(EXTENSIONS_DIR)/esp32_toolchain.cmake

$(EXTENSIONS_DIR)/micro_ros_dev/install:
	rm -rf micro_ros_dev; \
	mkdir micro_ros_dev; cd micro_ros_dev; \
	git clone -b rolling https://github.com/ament/ament_cmake src/ament/ament_cmake; \
	git clone -b rolling https://github.com/ament/ament_index src/ament/ament_index; \
	git clone -b rolling https://github.com/ament/ament_lint src/ament/ament_lint; \
	git clone -b rolling https://github.com/ament/ament_package src/ament/ament_package; \
	git clone -b rolling https://github.com/ament/googletest src/ament/googletest; \
	git clone -b rolling https://github.com/ros2/ament_cmake_ros src/ros2/ament_cmake_ros; \
	set -e; \
	rm -r src/ros2/ament_cmake_ros/rmw_test_fixture; \
	rm -r src/ros2/ament_cmake_ros/rmw_test_fixture_implementation; \
	colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=gcc;

# ros2/rosidl needs to be pinned to an older version as ros2/rosidl#942 added rosidl_buffer as
# a dependency for various rosidl packages and we can't build it currently.
$(EXTENSIONS_DIR)/micro_ros_src/src:
	rm -rf micro_ros_src; \
	mkdir micro_ros_src; cd micro_ros_src; \
	if [ "$(MIDDLEWARE)" = "embeddedrtps" ]; then \
		git clone -b main https://github.com/micro-ROS/embeddedRTPS src/micro-ROS/embeddedRTPS; \
		git clone -b main https://github.com/micro-ROS/rmw_embeddedrtps src/micro-ROS/rmw_embeddedrtps; \
	else \
		git clone -b ros2 https://github.com/eProsima/Micro-XRCE-DDS-Client src/eProsima/Micro-XRCE-DDS-Client; \
		git clone -b rolling https://github.com/micro-ROS/rmw_microxrcedds src/micro-ROS/rmw_microxrcedds; \
	fi; \
	git clone -b ros2 https://github.com/eProsima/micro-CDR src/eProsima/micro-CDR; \
	git clone -b rolling https://github.com/micro-ROS/micro_ros_msgs src/micro-ROS/micro_ros_msgs; \
	git clone -b rolling https://github.com/micro-ROS/micro_ros_utilities src/micro-ROS/micro_ros_utilities; \
	git clone -b upstream-patches https://github.com/micro-ROS/rcl src/micro-ROS/rcl; \
	git clone -b rolling https://github.com/micro-ROS/rcutils src/micro-ROS/rcutils; \
	git clone -b rolling https://github.com/micro-ROS/rosidl_typesupport src/micro-ROS/rosidl_typesupport; \
	git clone -b rolling https://github.com/micro-ROS/rosidl_typesupport_microxrcedds src/micro-ROS/rosidl_typesupport_microxrcedds; \
	git clone -b rolling https://github.com/ros2/common_interfaces src/ros2/common_interfaces; \
	git clone -b rolling https://github.com/ros2/example_interfaces src/ros2/example_interfaces; \
	git clone -b rolling https://github.com/ros2/rcl_interfaces src/ros2/rcl_interfaces; \
	git clone -b rolling https://github.com/ros2/rcl_logging src/ros2/rcl_logging; \
	git clone -b rolling https://github.com/ros2/rclc src/ros2/rclc; \
	git clone -b rolling https://github.com/ros2/rmw src/ros2/rmw; \
	git clone -b rolling https://github.com/ros2/rmw_implementation src/ros2/rmw_implementation; \
	git clone -b rolling https://github.com/ros2/ros2_tracing src/ros2/ros2_tracing; \
	git clone -b rolling https://github.com/ros2/rosidl src/ros2/rosidl; \
	cd src/ros2/rosidl; \
	git reset --hard 5f4ace0288ecf942307ed62b9239ab5986884676; \
	cd ../../..; \
	git clone -b rolling https://github.com/ros2/rosidl_core src/ros2/rosidl_core; \
	git clone -b rolling https://github.com/ros2/rosidl_defaults src/ros2/rosidl_defaults; \
	git clone -b rolling https://github.com/ros2/rosidl_dynamic_typesupport src/ros2/rosidl_dynamic_typesupport; \
	git clone -b rolling https://github.com/ros2/test_interface_files src/ros2/test_interface_files; \
	git clone -b rolling https://github.com/ros2/unique_identifier_msgs src/ros2/unique_identifier_msgs; \
	set -e; \
	rm -r src/micro-ROS/rcl/rcl_lifecycle; \
	rm -r src/micro-ROS/rcl/rcl_yaml_param_parser; \
	rm -r src/micro-ROS/rosidl_typesupport/rosidl_typesupport_tests; \
	rm -r src/micro-ROS/rosidl_typesupport_microxrcedds/test; \
	rm -r src/ros2/common_interfaces/common_interfaces; \
	rm -r src/ros2/common_interfaces/sensor_msgs_py; \
	rm -r src/ros2/rcl_logging/rcl_logging_implementation; \
	rm -r src/ros2/rcl_logging/rcl_logging_spdlog; \
	rm -r src/ros2/rclc/rclc_examples; \
	rm -r src/ros2/rclc/rclc_lifecycle; \
	rm -r src/ros2/rmw/rmw_security_common; \
	rm -r src/ros2/rmw_implementation/test_rmw_implementation; \
	rm -r src/ros2/ros2_tracing/lttngpy; \
	rm -r src/ros2/ros2_tracing/ros2trace; \
	rm -r src/ros2/ros2_tracing/test_ros2trace; \
	rm -r src/ros2/ros2_tracing/test_tracetools; \
	rm -r src/ros2/ros2_tracing/test_tracetools_launch; \
	rm -r src/ros2/ros2_tracing/tracetools_launch; \
	rm -r src/ros2/ros2_tracing/tracetools_read; \
	rm -r src/ros2/ros2_tracing/tracetools_test; \
	rm -r src/ros2/ros2_tracing/tracetools_trace; \
	rm -r src/ros2/rosidl/rosidl_buffer; \
	rm -r src/ros2/rosidl/rosidl_buffer_backend; \
	rm -r src/ros2/rosidl/rosidl_buffer_backend_registry; \
	rm -r src/ros2/rosidl/rosidl_generator_tests; \
	rm -r src/ros2/rosidl/rosidl_typesupport_introspection_cpp; \
	rm -r src/ros2/rosidl/rosidl_typesupport_introspection_tests; \
	cp -rfL $(EXTRA_ROS_PACKAGES) src/extra_packages || :; \
	test -f src/extra_packages/extra_packages.repos && cd src/extra_packages && vcs import --input extra_packages.repos || :;


$(EXTENSIONS_DIR)/micro_ros_src/install: $(EXTENSIONS_DIR)/esp32_toolchain.cmake $(EXTENSIONS_DIR)/micro_ros_dev/install $(EXTENSIONS_DIR)/micro_ros_src/src
	cd $(UROS_DIR); \
	unset AMENT_PREFIX_PATH; \
	PATH="$(subst /opt/ros/$(ROS_DISTRO)/bin,,$(PATH))"; \
	. ../micro_ros_dev/install/local_setup.sh; \
	colcon build \
		--merge-install \
		--packages-ignore-regex=.*_cpp \
		--metas $(EXTENSIONS_DIR)/colcon.meta $(APP_COLCON_META) \
		--cmake-args \
		"--no-warn-unused-cli" \
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_TOOLCHAIN_FILE=$(EXTENSIONS_DIR)/esp32_toolchain.cmake \
		-DCMAKE_VERBOSE_MAKEFILE=OFF \
        -DIDF_INCLUDES='${IDF_INCLUDES}' \
		-DCMAKE_C_STANDARD=$(C_STANDARD) \
		-DUCLIENT_C_STANDARD=$(C_STANDARD);

$(EXTENSIONS_DIR)/libmicroros.a: $(EXTENSIONS_DIR)/micro_ros_src/install
	mkdir -p $(UROS_DIR)/libmicroros; cd $(UROS_DIR)/libmicroros; \
	for file in $$(find $(UROS_DIR)/install/lib/ -name '*.a'); do \
		folder=$$(echo $$file | sed -E "s/(.+)\/(.+).a/\2/"); \
		mkdir -p $$folder; cd $$folder; $(X_AR) x $$file; \
		for f in *; do \
			mv $$f ../$$folder-$$f; \
		done; \
		cd ..; rm -rf $$folder; \
	done ; \
	$(X_AR) rc -s libmicroros.a *.obj; cp libmicroros.a $(EXTENSIONS_DIR); \
	cd ..; rm -rf libmicroros; \
	cp -R $(UROS_DIR)/install/include $(EXTENSIONS_DIR)/include;
