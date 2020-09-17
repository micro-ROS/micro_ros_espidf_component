EXTENSIONS_DIR = $(shell pwd)
UROS_DIR = $(EXTENSIONS_DIR)/micro_ros_src
BUILD_DIR ?= $(EXTENSIONS_DIR)/build

DEBUG ?= 0

ifeq ($(DEBUG), 1)
	BUILD_TYPE = Debug
else
	BUILD_TYPE = Release
endif

CFLAGS_INTERNAL := $(CFLAGS) -ffunction-sections -fdata-sections
CXXFLAGS_INTERNAL := $(CXXFLAGS) -ffunction-sections -fdata-sections

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
		sed "s/@CMAKE_C_COMPILER@/$(subst /,\/,$(CC))/g" | \
		sed "s/@CMAKE_CXX_COMPILER@/$(subst /,\/,$(CXX))/g" | \
		sed "s/@CFLAGS@/$(subst /,\/,$(CFLAGS_INTERNAL))/g" | \
		sed "s/@CXXFLAGS@/$(subst /,\/,$(CXXFLAGS_INTERNAL))/g" | \
		sed "s/@IDF_PATH@/$(subst /,\/,$(IDF_PATH))/g" | \
		sed "s/@BUILD_CONFIG_DIR@/$(subst /,\/,$(BUILD_DIR)/config)/g" \
		> $(EXTENSIONS_DIR)/esp32_toolchain.cmake

$(EXTENSIONS_DIR)/micro_ros_dev/install:
	rm -rf micro_ros_dev; \
	mkdir micro_ros_dev; cd micro_ros_dev; \
	git clone -b foxy https://github.com/ament/ament_cmake src/ament_cmake; \
	git clone -b foxy https://github.com/ament/ament_lint src/ament_lint; \
	git clone -b foxy https://github.com/ament/ament_package src/ament_package; \
	git clone -b foxy https://github.com/ament/googletest src/googletest; \
	git clone -b foxy https://github.com/ros2/ament_cmake_ros src/ament_cmake_ros; \
	colcon build; 

$(EXTENSIONS_DIR)/micro_ros_src/src:
	rm -rf micro_ros_src; \
	mkdir micro_ros_src; cd micro_ros_src; \
	git clone -b foxy https://github.com/eProsima/micro-CDR src/micro-CDR; \
	git clone -b foxy https://github.com/eProsima/Micro-XRCE-DDS-Client src/Micro-XRCE-DDS-Client; \
	git clone -b foxy https://github.com/micro-ROS/rcl src/rcl; \
	git clone -b master https://github.com/micro-ROS/rclc src/rclc; \
	git clone -b foxy https://github.com/micro-ROS/rcutils src/rcutils; \
	git clone -b foxy https://github.com/micro-ROS/rmw-microxrcedds src/rmw-microxrcedds; \
	git clone -b foxy https://github.com/micro-ROS/rosidl_typesupport src/rosidl_typesupport; \
	git clone -b foxy https://github.com/micro-ROS/rosidl_typesupport_microxrcedds src/rosidl_typesupport_microxrcedds; \
	git clone -b master https://github.com/ros2/tinydir_vendor src/tinydir_vendor; \
	git clone -b foxy https://github.com/ros2/rosidl src/rosidl; \
	git clone -b foxy https://github.com/ros2/rmw src/rmw; \
	git clone -b foxy https://github.com/ros2/rcl_interfaces src/rcl_interfaces; \
	git clone -b foxy https://github.com/ros2/rosidl_defaults src/rosidl_defaults; \
	git clone -b foxy https://github.com/ros2/unique_identifier_msgs src/unique_identifier_msgs; \
	git clone -b foxy https://github.com/ros2/common_interfaces src/common_interfaces; \
	git clone -b foxy https://github.com/ros2/test_interface_files src/test_interface_files; \
	git clone -b foxy https://github.com/ros2/rmw_implementation src/rmw_implementation; \
	git clone -b foxy_microros https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing src/ros2_tracing; \
	touch src/rosidl/rosidl_typesupport_introspection_c/COLCON_IGNORE; \
    touch src/rosidl/rosidl_typesupport_introspection_cpp/COLCON_IGNORE; \
    touch src/rclc/rclc_examples/COLCON_IGNORE; \
	touch src/rcl/rcl_yaml_param_parser/COLCON_IGNORE; \
	cp -f ../serial_transport_external/esp32_serial_transport.c src/Micro-XRCE-DDS-Client/src/c/profile/transport/serial/serial_transport_external.c; \
    cp -f ../serial_transport_external/esp32_serial_transport.h src/Micro-XRCE-DDS-Client/include/uxr/client/profile/transport/serial/serial_transport_external.h;
	

$(EXTENSIONS_DIR)/micro_ros_src/install: $(EXTENSIONS_DIR)/esp32_toolchain.cmake $(EXTENSIONS_DIR)/micro_ros_dev/install $(EXTENSIONS_DIR)/micro_ros_src/src
	cd $(UROS_DIR); \
	. ../micro_ros_dev/install/local_setup.sh; \
	colcon build \
		--merge-install \
		--packages-ignore-regex=.*_cpp \
		--metas $(EXTENSIONS_DIR)/colcon.meta \
		--cmake-args \
		"--no-warn-unused-cli" \
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_TOOLCHAIN_FILE=$(EXTENSIONS_DIR)/esp32_toolchain.cmake \
		-DCMAKE_VERBOSE_MAKEFILE=OFF; \

$(EXTENSIONS_DIR)/libmicroros.a: $(EXTENSIONS_DIR)/micro_ros_src/install
	mkdir -p $(UROS_DIR)/libmicroros; cd $(UROS_DIR)/libmicroros; \
	for file in $$(find $(UROS_DIR)/install/lib/ -name '*.a'); do \
		folder=$$(echo $$file | sed -E "s/(.+)\/(.+).a/\2/"); \
		mkdir -p $$folder; cd $$folder; $(AR) x $$file; \
		for f in *; do \
			mv $$f ../$$folder-$$f; \
		done; \
		cd ..; rm -rf $$folder; \
	done ; \
	$(AR) rc libmicroros.a *.obj; cp libmicroros.a $(EXTENSIONS_DIR); ranlib $(EXTENSIONS_DIR)/libmicroros.a; \
	cd ..; rm -rf libmicroros; \
	cp -R $(UROS_DIR)/install/include $(EXTENSIONS_DIR)/include;