#!/bin/bash

set -eu

sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 
sudo apt update -q
sudo apt install -yq python3-colcon-common-extensions
source $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser empy

set +u
