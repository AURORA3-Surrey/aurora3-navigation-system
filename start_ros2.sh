#!/bin/bash
# run on turtlebot to start ros2 (leave running in its own terminal)
# chmod +x start_turtlebot.sh
# ./start_turtlebot.sh
set -e

export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "Launching robot"
echo ""

ros2 launch turtlebot3_bringup robot.launch.py
