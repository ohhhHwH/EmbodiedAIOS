#!/bin/bash

# TODO: ros2 run will failed now
# colcon build --packages-select piper_raspi_hc_sr04
# source install/setup.bash
# ros2 run piper_raspi_hc_sr04 hc_sr04_node


export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/wheatfox/code/EmbodiedAIOS/src/piper_raspi_hc_sr04/fastrtps.xml
export ROS_LOCALHOST_ONLY=0
export ROS_IP=$(hostname -I| awk '{for(i=1;i<=NF;i++)if($i~/^100/)print $i}')

source /opt/ros/humble/setup.bash

echo "We are using tailscale ip: $ROS_IP"
# python3 raspi_hc_sr04/node.py

# create a default talker
ros2 topic pub /chatter std_msgs/String "data: 'Hello World from Raspberry Pi :)'"