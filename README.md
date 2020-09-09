# micro-ROS component for ESP-IDF

## Dependencies

This componentes needs `colcon` in order to build micro-ROS packages:

<!-- apt install lsb-release git -->
```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Some pyhton3 packages are also required inside the IDF virtual environment:

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

