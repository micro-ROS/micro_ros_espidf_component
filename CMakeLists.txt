idf_component_register(SRCS "network_interfaces/uros_ethernet_netif.c" "network_interfaces/uros_wlan_netif.c"
                       INCLUDE_DIRS "network_interfaces"
                       REQUIRES nvs_flash esp_wifi esp_eth lwip)

if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
set(submake "make")
else()
set(submake "make")
endif()

if (${IDF_VERSION_MAJOR} EQUAL 5)
    set(CMAKE_C_STANDARD 17)
else()
    set(CMAKE_C_STANDARD 11)
endif()

set(APP_COLCON_META "${PROJECT_DIR}/app-colcon.meta")
if(NOT EXISTS "${APP_COLCON_META}")
set(APP_COLCON_META "")
endif()

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

# Create a string with all packages with a for loop
foreach(package ${PACKAGES})
    string(APPEND IDF_INCLUDES "${${package}_includes_string}")
endforeach()

execute_process(
    WORKING_DIRECTORY ${COMPONENT_DIR}
    RESULT_VARIABLE libmicroros_ret
    COMMAND
        ${submake} -j -f libmicroros.mk
                X_CC=${CMAKE_C_COMPILER}
                X_AR=${CMAKE_AR}
                X_STRIP=${CMAKE_STRIP}
                X_CFLAGS=${CMAKE_C_FLAGS}
                X_CXX=${CMAKE_CXX_COMPILER}
                X_CXXFLAGS=${CMAKE_CXX_FLAGS}
                C_STANDARD=${CMAKE_C_STANDARD}
                MIDDLEWARE=${MIDDLEWARE}
                BUILD_DIR=${CMAKE_BINARY_DIR}
                IDF_INCLUDES=${IDF_INCLUDES}
                IDF_PATH=${IDF_PATH}
                IDF_TARGET=${IDF_TARGET}
                APP_COLCON_META=${APP_COLCON_META}
                IDF_VERSION_MAJOR=${IDF_VERSION_MAJOR}
                IDF_VERSION_MINOR=${IDF_VERSION_MINOR}
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

file(GLOB include_folders LIST_DIRECTORIES true CONFIGURE_DEPENDS ${COMPONENT_DIR}/include/*)
foreach(include_folder ${include_folders})
    if(IS_DIRECTORY ${include_folder})
        get_filename_component(folder_name ${include_folder} NAME)
        target_include_directories(libmicroros-prebuilt INTERFACE ${COMPONENT_DIR}/include/${folder_name})
    endif()
endforeach()

add_dependencies(${COMPONENT_LIB} libmicroros-prebuilt)
target_link_libraries(${COMPONENT_LIB} INTERFACE libmicroros-prebuilt)

set_directory_properties( PROPERTIES ADDITIONAL_CLEAN_FILES
    "${COMPONENT_DIR}/include;${COMPONENT_DIR}/micro_ros_dev;${COMPONENT_DIR}/micro_ros_src;${COMPONENT_DIR}/esp32_toolchain.cmake" )

add_custom_target(clean-microros COMMAND ${submake} -f libmicroros.mk clean
                            WORKING_DIRECTORY ${COMPONENT_DIR})
