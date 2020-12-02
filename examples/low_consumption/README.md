Required Change in configuration:
	CONFIG_PM_ENABLE=y
	CONFIG_PM_DFS_INIT_AUTO=y
	CONFIG_ESP_MAIN_TASK_STACK_SIZE=3000
	CONFIG_FREERTOS_USE_TICKLESS_IDLE=y


ESP32 (WRover Module, 16MByte Flash, 8MByte PSRAM):
	The VCC current is pulsating between min and max value.

	wifi listen_interval = 5
	ros2 topic send interval = 1000 ms

	The mean current (estimated) is: 26 mA

ESP32-S2 (WRover Module, 4MByte Flash, 8MByte PSRAM):
	The VCC current is pulsating between min and max value.

	wifi listen_interval = 5
	ros2 topic send interval = 1000 ms

	The mean current (estimated) is: 14 mA
			
	By inreasing the wifi listen_interval (e.g. 30) and 
	the topic send interval (e.g. 60000), the mean 
	current might be reduced to 5 mA.
