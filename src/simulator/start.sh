#!/bin/bash

# simulator

source ip

ps1="SIMULATOR::"$simulator_ip" "
export PS1=$ps1

source /usr/share/gazebo/setup.sh
# gazebo_master_uri="http://"$subscriber_ip":11345"
gazebo_master_uri="http://subscriber:11345"
export GAZEBO_MASTER_URI=$gazebo_master_uri

cd /root/gzweb
npm start &
