#!/bin/bash

# subscriber

source ip

ps1="SUBSCRIBER::"$subscriber_ip" "
export PS1=$ps1

source /usr/share/gazebo/setup.sh

colcon build --symlink-install
source ./install/setup.bash

ln -sf /root/workspace/src/subscriber/models/maze_1 /usr/share/gazebo-9/models/

export TURTLEBOT3_MODEL=waffle_pi
ros2 launch subscriber empty_world.launch.py &
