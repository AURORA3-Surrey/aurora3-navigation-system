#!/bin/bash
# run on turtlebot to start ros2 (leave running in its own terminal)
# chmod +x start_turtlebot.sh
# ./start_turtlebot.sh
set -e
source /opt/ros/jazzy/setup.bash

export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=77

ROBOT_IP=$(ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n1 || true)

echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Robot IP (wlan0): ${ROBOT_IP:-NOT FOUND}"
echo "Launching robot"
echo ""

ros2 launch turtlebot3_bringup robot.launch.py