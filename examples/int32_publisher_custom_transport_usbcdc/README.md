
# USB-CDC Custom Transport Example

| Supported Targets | ESP32-S2 | ESP32-S3 |
|-------------------|----------|----------|

This example demonstrates how to set up the ESP32-S2/S3 to function as a USB Serial Device (CDC-ACM) and communicate with micro-ROS agent using USB-CDC custom transport.

The [TinyUSB component](https://components.espressif.com/components/espressif/esp_tinyusb) is used as the USB stack.

This example is based on the [int32_publisher_custom_transport](https://github.com/micro-ROS/micro_ros_espidf_component/tree/jazzy/examples/int32_publisher_custom_transport), the [TinyUSB Serial Device Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_serial_device), and the [TinyUSB Console Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_console) for log output.

## How to use example

This example is configured to use the two interfaces of USB-CDC. One interface is used for the micro-ROS communication, and the other interface is used for the log output.

### Hardware Required

This example can be run on any development board that has a USB-CDC interface.

### Configure the project

Set the target device in the project configuration:

```bash
idf.py set-target esp32s2 # or esp32s3
```

If you want to use only the micro-ROS communication interface, you need to turn off log output in menuconfig. Run `idf.py menuconfig` and navigate to `Component config → Log output → Default log verbosity` and set it to `No output`. You should also set `Component config → TinyUSB Stack → Communication Device Class (CDC) → CDC Channel Count` to 1.

### Build and Flash

> [!NOTE]
> The ESP32-S2/S3 chip needs to be in bootloader mode before it can be detected as a DFU device and flash. This can be achieved by pulling GPIO0 down (e.g., pressing the BOOT button), pulling RESET down for a moment, and releasing GPIO0.

#### Build the project

Build DFU image:

```bash
idf.py dfu
```

#### Flash the project

Put the ESP32-S2/S3 into bootloader mode and run the following command:

```bash
idf.py dfu-flash
```

### Run micro-ROS Agent

```bash
export ROS_DOMAIN_ID=100 # Set the ROS2 domain ID
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Output expected:

```bash
[1724443525.673894] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1724443525.674071] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
[1724443529.936542] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1724443531.062646] info     | Root.cpp           | create_client            | create                 | client_key: 0x3E801A05, session_id: 0x81
[1724443531.062805] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x3E801A05, address: 0
[1724443531.107532] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x3E801A05, participant_id: 0x000(1)
[1724443531.137064] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x3E801A05, topic_id: 0x000(2), participant_id: 0x000(1)
[1724443531.167351] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x3E801A05, publisher_id: 0x000(3), participant_id: 0x000(1)
[1724443531.237811] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x3E801A05, datawriter_id: 0x000(5), publisher_id: 0x000(3)
```

After connecting the ESP32-S2/S3 and the micro-ROS agent, you can list the topics:

```bash
export ROS_DOMAIN_ID=100 # Set the ROS2 domain ID
ros2 topic list
```

Output expected:

```bash
/esp32s2/int32_publisher_usbcdc
/parameter_events
/rosout
```

And see if the `esp32s2/int32_publisher_usbcdc` topic is available. You can echo the topic to see the messages:

```bash
export ROS_DOMAIN_ID=100 # Set the ROS2 domain ID
ros2 topic echo esp32s2/int32_publisher_usbcdc
```

Output expected:

```bash
data: 1
---
data: 2
---
data: 3
---
data: 4
---
data: 5
.
.
.
```

To see the log output, you can use the following command:

```bash
minicom -D /dev/ttyACM1 -b 115200
```

Output expected:

```bash
Welcome to minicom 2.8

OPTIONS: I18n
Port /dev/ttyACM1
Press CTRL-A Z for help on special keys

I (2688) MAIN: micro-ROS task created
I (2688) main_task: Returned from app_main()
I (3708) TIMER_CALLBACK: Message published: 0
I (4708) TIMER_CALLBACK: Message published: 1
I (5708) TIMER_CALLBACK: Message published: 2
I (6708) TIMER_CALLBACK: Message published: 3
I (7708) TIMER_CALLBACK: Message published: 4
I (8708) TIMER_CALLBACK: Message published: 5
.
.
.
```
