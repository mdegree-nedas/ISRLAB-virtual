#!/bin/bash

# publisher

source ip

ps1="PUBLISHER::"$publisher_ip" "
export PS1=$ps1

colcon build --symlink-install
source ./install/setup.bash

ros2 run publisher main
