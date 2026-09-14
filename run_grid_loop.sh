#!/bin/bash
# LOOP FOR 15/09/26 SOFT OPENING VAR DEMO
# do chmod +x run_grid_loop.sh once and run with ./run_grid_loop.sh
# 3x3 grid on the 3.0 x 0.8 m tape rectangle (start the robot on a corner of the tape with 3m side on its right)
# to change topic --vicon-topic /vicon/Other/Name
# change pause length with LOOP_SLEEP_SECS=10 ./run_grid_loop.sh
# for faster (not recommended with turtlebot3) laps do ./run_grid_loop.sh --max-speed >0.15
# ctrl-C stops the whole loop

set -u

SLEEP_SECS="${LOOP_SLEEP_SECS:-5}"
NODE_PID=""

set +u
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}" || { echo "ROS 2 jazzy setup.bash not found"; exit 1; }
set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-10.78.24.110}"

cd "$(dirname "$0")" || exit 1
[ -f vicon_grid.py ] || { echo "vicon_grid.py not found next to run_grid_loop.sh - sync it first"; exit 1; }

run_count=0

# ctrl-c kill handling
cleanup() {
    trap '' INT TERM
    pid="${NODE_PID}"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        echo
        echo "loop: stopping grid node (pid ${pid}) - zero-twist burst incoming..."
        kill -INT "$pid" 2>/dev/null
        tries=0
        while kill -0 "$pid" 2>/dev/null && [ "$tries" -lt 10 ]; do
            sleep 0.5; tries=$((tries + 1))
        done
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null
            tries=0
            while kill -0 "$pid" 2>/dev/null && [ "$tries" -lt 6 ]; do
                sleep 0.5; tries=$((tries + 1))
            done
        fi
        if kill -0 "$pid" 2>/dev/null; then
            echo "loop: node did not exit - killing it hard"
            kill -KILL "$pid" 2>/dev/null
        fi
        wait "$pid" 2>/dev/null
    fi
    # force the robot to zero velocity from the shell
    if [ "${FINAL_STOP_PUB:-1}" = "1" ] && command -v ros2 >/dev/null 2>&1; then
        echo "loop: publishing 4x zero /cmd_vel from the shell as final stop..."
        timeout 10 ros2 topic pub --times 4 --rate 2 /cmd_vel \
            geometry_msgs/msg/Twist \
            '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
            >/dev/null 2>&1 || true
    fi
    echo "loop: stopped after ${run_count} run(s)"
    exit 0
}
trap cleanup INT TERM

while true; do
    run_count=$((run_count + 1))
    echo "=================================================================="
    echo " GRID LOOP - run #${run_count}   (ctrl-C quits | if bad bad: emergencystop.py)"
    echo "=================================================================="
    # DEMO taped rectangle: 3 rows x 3 cols, 1.5 m along heading x 0.4 m to the right
    python3 vicon_grid.py --grid-size 3 --grid-cols 3 --cell-size-x 1.5 --cell-size-y 0.4 --max-speed 0.15 --turn-speed 0.2 "$@" &
    NODE_PID=$!
    wait "$NODE_PID"
    status=$?
    NODE_PID=""
    if [ "$status" -eq 130 ] || [ "$status" -eq 143 ]; then
        echo "loop: run #${run_count} was interrupted (status ${status})"
        break
    fi
    if [ "$status" -ne 0 ]; then
        break
    fi
    echo " run #${run_count} done - next run in ${SLEEP_SECS} s (Ctrl-C quits)"
    sleep "${SLEEP_SECS}"
done

echo "last run exited with status ${status:-?}, put the robot back on a tape corner"
echo "(facing along the 3 m side, tape to its right), then ./run_grid_loop.sh."
