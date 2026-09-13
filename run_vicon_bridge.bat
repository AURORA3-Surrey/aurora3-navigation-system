@echo off
:: VICON must be running first (netstat -an shows a line with :801 LISTENING)
:: hostname:=localhost is correct while VICON runs on the Windows machine
:: ROS_DOMAIN_ID=77 has to match robot (~/.bashrc and in start_ros2.sh)
:: required on Windows VICON machine: .wslconfig networkingMode=mirrored + Hyper-V firewall inbound Allow

title ROS 2 Jazzy - Vicon Receiver

echo.
echo ==========================================
echo ROS 2 Jazzy Vicon Receiver
echo ROS_DOMAIN_ID=77
echo ==========================================
echo.
echo Starting WSL Ubuntu 24.04...
echo.

wsl.exe -d Ubuntu-24.04 -- bash -ic "cd /home/aurora3var/vicon_receiver_ws && source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=77 && echo '[1/3] Building workspace...' && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && echo '[2/3] Sourcing workspace...' && source install/setup.bash && echo '[3/3] Launching Vicon receiver...' && ros2 launch vicon_receiver client.launch.py hostname:=localhost topic_namespace:=vicon buffer_size:=200 world_frame:=map vicon_frame:=vicon map_xyz:='[0.0, 0.0, 0.0]' map_rpy:='[0.0, 0.0, 0.0]' map_rpy_in_degrees:=false"

echo.
echo ==========================================
echo Vicon receiver stopped
echo ==========================================
echo.

pause