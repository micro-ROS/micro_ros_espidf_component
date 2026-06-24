idf_component_register(SRCS "esp32s2_usbcdc_transport.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component # include <uxr/client/transport.h>
)
