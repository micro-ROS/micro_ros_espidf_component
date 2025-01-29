![banner](.images/banner-dark-theme.png#gh-dark-mode-only)
![banner](.images/banner-light-theme.png#gh-light-mode-only)

# micro-ROS component for ESP-IDF

This component has been tested in ESP-IDF v4.4, and v5.2 with ESP32, ESP32-S2, ESP32-S3, ESP32-C3 and ESP32-C6.

## Dependencies

This component needs `colcon` and other Python 3 packages inside the IDF virtual environment in order to build micro-ROS packages:

```bash
. $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser colcon-common-extensions
```

## Middlewares available

This package support the usage of micro-ROS on top of two different middlewares:
- [eProsima Micro XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/): the default micro-ROS middleware.
- [embeddedRTPS](https://github.com/embedded-software-laboratory/embeddedRTPS): an experimental implementation of a RTPS middleware compatible with ROS 2.

In order to select it, use `idf.py menuconfig` and go to `micro-ROS Settings > micro-ROS middleware`
## Usage

You can clone this repo directly in the `components` folder of your project.

If you encounter issues during the build process, ensure that you are running in a clean shell environment _without_ the ROS 2 setup script sourced.

## Example

In order to test a int32_publisher example:

```bash
. $IDF_PATH/export.sh
cd examples/int32_publisher
# Set target board [esp32|esp32s2|esp32s3|esp32c3]
idf.py set-target esp32
idf.py menuconfig
# Set your micro-ROS configuration and WiFi credentials under micro-ROS Settings
idf.py build
idf.py flash
idf.py monitor
```

To clean and rebuild all the micro-ROS library:

```bash
idf.py clean-microros
```

Is possible to use a micro-ROS Agent just with this docker command:

```bash
# UDPv4 micro-ROS Agent
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

## Build with docker container

It's possible to build this example application using preconfigured docker container. Execute this line to build an example app using docker container:

```bash
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/micro_ros_espidf_component -v  /dev:/dev --privileged --workdir /micro_ros_espidf_component microros/esp-idf-microros:latest /bin/bash  -c "cd examples/int32_publisher; idf.py menuconfig build flash monitor"
```

Dockerfile for this container is provided in the ./docker directory and available in dockerhub. This approach uses ESP-IDF v5.

## Using serial transport

By default, micro-ROS component uses UDP transport, but is possible to enable UART transport or any other custom transport setting the `colcon.meta` like:

```json
...
"rmw_microxrcedds": {
    "cmake-args": [
        ...
        "-DRMW_UXRCE_TRANSPORT=custom",
        ...
    ]
},
...
```

An example on how to implement this external transports is available in `examples/int32_publisher_custom_transport`.

Available ports are `0`, `1` and `2` corresponding `UART_NUM_0`, `UART_NUM_1` and `UART_NUM_2`.

Is possible to use a micro-ROS Agent just with this docker command:

```bash
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev [YOUR BOARD PORT] -v6
```

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

There are no known limitations.
