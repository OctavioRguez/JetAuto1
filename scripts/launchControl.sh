#!/bin/bash

# Stop the JetAuto app service
sudo systemctl stop start_app_node.service

cd ~/catkin_ws/src/jetauto1/scripts
chmod +x main.py

# Start the JetAuto controller (launch only available inside the JetAuto)
cd 
trap 'kill $PIDlaunch; wait "$PIDlaunch"; exit 1' SIGINT
source ~/jetauto_ws/devel/setup.bash
roslaunch jetauto_controller jetauto_controller.launch &
PIDlaunch=$!
echo $PIDlaunch
sleep 6

trap 'wait "$PIDrun"' SIGINT
source ~/catkin_ws/devel/setup.bash
rosrun jetauto1 main.py &
PIDrun=$!
echo $PIDrun
sleep 4

while true; do
    sleep 1
done