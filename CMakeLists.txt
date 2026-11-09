option(BUILD_ETHERNET_INTERFACE "Build ethernet interface support" OFF)
option(BUILD_WLAN_INTERFACE "Build wlan interface support" ON)

set(COMPONENT_SRC "posix_functions/nanosleep.c")

if(BUILD_ETHERNET_INTERFACE)
    list(APPEND COMPONENT_SRC "network_interfaces/uros_ethernet_netif.c")
endif()

if(BUILD_WLAN_INTERFACE)
    list(APPEND COMPONENT_SRC "network_interfaces/uros_wlan_netif.c")
endif()

idf_component_register(SRCS ${COMPONENT_SRC}
                       INCLUDE_DIRS "network_interfaces"
                       REQUIRES nvs_flash esp_wifi esp_eth lwip)

if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
set(submake "make")
else()
set(submake "make")
endif()

set(CMAKE_C_STANDARD 17)

# Detect creation/removal as well as edits of the optional application metadata.
file(GLOB APP_COLCON_META CONFIGURE_DEPENDS "${PROJECT_DIR}/app-colcon.meta")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${COMPONENT_DIR}/colcon.meta" ${APP_COLCON_META})

set(EXTRA_ROS_PACKAGES "${PROJECT_DIR}/extra_ros_packages")
if(NOT EXISTS "${EXTRA_ROS_PACKAGES}")
set(EXTRA_ROS_PACKAGES "${COMPONENT_DIR}/extra_packages")
endif()

if(${CONFIG_MICRO_ROS_ESP_EMBEDDEDRTPS_MIDDLEWARE})
message(STATUS "Using embeddedRTPS middleware")
set(MIDDLEWARE "embeddedrtps")
else()
message(STATUS "Using Micro XRCE-DDS middleware")
set(MIDDLEWARE "microxrcedds")
endif()

# Create a list of packages to get includes
idf_build_get_property(PACKAGES BUILD_COMPONENTS)

# Add includes for each package
foreach(package ${PACKAGES})
    idf_component_get_property(${package}_dir ${package} COMPONENT_DIR)
    idf_component_get_property(${package}_includes ${package} INCLUDE_DIRS)
    list(TRANSFORM ${package}_includes PREPEND " -I${${package}_dir}/")
    string(REPLACE ";" "" ${package}_includes_string "${${package}_includes}")
endforeach()

# Workaround for https://github.com/ros2/rosidl/pull/739
string(APPEND IDF_INCLUDES "-I${COMPONENT_DIR}/include_override")

# Create a string with all packages with a for loop
foreach(package ${PACKAGES})
    string(APPEND IDF_INCLUDES "${${package}_includes_string}")
endforeach()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/microros_flags.cmake)
microros_get_flags(C MICROROS_C_FLAGS)
microros_get_flags(CXX MICROROS_CXX_FLAGS)
idf_build_get_property(IDF_TARGET_ARCH IDF_TARGET_ARCH)
set(BUILD_CONFIG_DIR "${CMAKE_BINARY_DIR}/config")
file(SHA256 "${BUILD_CONFIG_DIR}/sdkconfig.h" MICROROS_SDKCONFIG_HASH)
set(MICROROS_APP_META_HASH "")
if(APP_COLCON_META)
    file(SHA256 "${APP_COLCON_META}" MICROROS_APP_META_HASH)
endif()
# configure_file only updates the output when its contents change. Generate it
# here rather than substituting compiler flags through Make and sed.
configure_file(esp32_toolchain.cmake.in "${COMPONENT_DIR}/esp32_toolchain.cmake" @ONLY)

execute_process(
    WORKING_DIRECTORY ${COMPONENT_DIR}
    RESULT_VARIABLE libmicroros_ret
    COMMAND
        ${submake} -j -f libmicroros.mk
                X_AR=${CMAKE_AR}
                C_STANDARD=${CMAKE_C_STANDARD}
                MIDDLEWARE=${MIDDLEWARE}
                BUILD_DIR=${CMAKE_BINARY_DIR}
                APP_COLCON_META=${APP_COLCON_META}
                EXTRA_ROS_PACKAGES=${EXTRA_ROS_PACKAGES}
)
if(libmicroros_ret AND NOT libmicroros_ret EQUAL 0)
  message(FATAL_ERROR "FAILED: ${libmicroros_ret}")
endif()

add_prebuilt_library(libmicroros-prebuilt ""
                     REQUIRES lwip)

add_dependencies(libmicroros-prebuilt libmicroros_project)

set_target_properties(libmicroros-prebuilt PROPERTIES IMPORTED_LOCATION ${COMPONENT_DIR}/libmicroros.a)
target_include_directories(libmicroros-prebuilt INTERFACE ${COMPONENT_DIR}/include)
target_compile_definitions(libmicroros-prebuilt INTERFACE RCL_MICROROS)

file(GLOB include_folders LIST_DIRECTORIES true CONFIGURE_DEPENDS ${COMPONENT_DIR}/include/*)
foreach(include_folder ${include_folders})
    if(IS_DIRECTORY ${include_folder})
        get_filename_component(folder_name ${include_folder} NAME)
        target_include_directories(libmicroros-prebuilt INTERFACE ${COMPONENT_DIR}/include/${folder_name})
    endif()
endforeach()

target_compile_definitions(${COMPONENT_LIB} PUBLIC RCL_MICROROS)
add_dependencies(${COMPONENT_LIB} libmicroros-prebuilt)
target_link_libraries(${COMPONENT_LIB} INTERFACE libmicroros-prebuilt)

# libmicroros.a is linked after this component, so force the linker to pull
# in the nanosleep() shim before rclc references it
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u nanosleep")

set_directory_properties( PROPERTIES ADDITIONAL_CLEAN_FILES
    "${COMPONENT_DIR}/include;${COMPONENT_DIR}/micro_ros_dev;${COMPONENT_DIR}/micro_ros_src;${COMPONENT_DIR}/esp32_toolchain.cmake" )

add_custom_target(clean-microros COMMAND ${submake} -f libmicroros.mk clean
                            WORKING_DIRECTORY ${COMPONENT_DIR})
