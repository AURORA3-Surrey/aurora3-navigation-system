#!/bin/bash
# do chmod +x run_grid_loop.sh once and run with ./run_grid_loop.sh
# to change topic --vicon-topic /vicon/Other/Name
# change pause length with LOOP_SLEEP_SECS=10 ./run_grid_loop.sh
# ctrl-C stops the whole loop

set -u

SLEEP_SECS="${LOOP_SLEEP_SECS:-5}"

source /opt/ros/jazzy/setup.bash || { echo "ROS 2 jazzy setup.bash not found"; exit 1; }
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-10.78.24.110}"

cd "$(dirname "$0")" || exit 1
[ -f vicon_grid.py ] || { echo "vicon_grid.py not found next to run_grid_loop.sh - sync it first"; exit 1; }

run_count=0
trap 'echo; echo "loop: stopped by Ctrl-C after ${run_count} run(s)"; exit 0' INT TERM

while true; do
    run_count=$((run_count + 1))
    echo "=================================================================="
    echo " GRID LOOP - run #${run_count}   (ctrl-C quits | if bad bad: emergencystop.py)"
    echo "=================================================================="
    # failed run ends the loop
    python3 vicon_grid.py --grid-size 4 --grid-cols 2 --cell-size 0.25  --max-speed 0.06 --turn-speed 0.2 "$@" || break
    echo " run #${run_count} done - next run in ${SLEEP_SECS} s (Ctrl-C quits)"
    sleep "${SLEEP_SECS}"
done

echo "last run exited, reposition the robot then ./run_grid_loop.sh."
