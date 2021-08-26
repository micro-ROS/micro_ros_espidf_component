#!/bin/bash

set -eu

sudo apt update -q
sudo apt install -yq python3-pip
source $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources

set +u
