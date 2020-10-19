# micro-ROS component for ESP-IDF

This component has been tested in ESP-IDF v4.1

## Dependencies

This componentes needs `colcon` in order to build micro-ROS packages:

<!-- apt install lsb-release git -->
```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Some python3 packages are also required inside the IDF virtual environment:

```bash
. $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser empy
```

## Usage

```bash
. $IDF_PATH/export.sh
idf.py menuconfig
# Set your micro-ROS configuration and WiFi credentials
idf.py build 
idf.py flash 
idf.py monitor 
```

Is possible to use a micro-ROS Agent just with this docker command:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888 -v6
```

## Build with docker container

It's possible to build this example application using preconfigured docker container. Execute this line to build an example app using docker container:

```bash
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):$(pwd) --workdir $(pwd) microros/esp-idf-microros:latest /bin/bash  -c "idf.py build"
```

Docker image microros/esp-idf-microros:latest will be automatically pulled from hub.docker.com
and used to build an application.

After build is finished build results will be accessible in a local ./build directory. 

Dockerfile for this container is provided in the ./docker directory.

## Using serial transport

By default, micro-ROS component uses UDP transport, but is possible to enable UART transport setting the `colcon.meta` like:

```json
...
"rmw_microxrcedds": {
    "cmake-args": [
        ...
        "-DRMW_UXRCE_TRANSPORT=custom_serial",
        "-DRMW_UXRCE_DEFAULT_SERIAL_DEVICE=1",
        ...
    ]
},
...
```

Available ports are `0`, `1` and `2` corresponding `UART_NUM_0`, `UART_NUM_1` and `UART_NUM_2`.

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
