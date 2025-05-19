#!/bin/bash
# TODO: ros2 run will failed now
# colcon build --packages-select piper_raspi_hc_sr04
# source install/setup.bash
# ros2 run piper_raspi_hc_sr04 hc_sr04_node


export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_NAMESPACE=""
export ROS_DISCOVERY_SERVER=""
export ROS_LOCALHOST_ONLY=0

ip_address=$(hostname -I)
# get the ip start with 10, which is school network
ip_address=$(echo $ip_address | awk '{for(i=1;i<=NF;i++)if($i~/^10/)print $i}')

export ROS_IP=$ip_address

echo "ROS_IP: $ROS_IP"
python3 piper_raspi_hc_sr04/node.py