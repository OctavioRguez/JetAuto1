#!/bin/bash

# ./development_ws/src/module1/scripts/launchControl.sh
sudo systemctl stop start_app_node.service

cd ~/development_ws/src/module1/scripts
chmod +x main.py

cd 
trap 'kill $PIDlaunch; wait "$PIDlaunch"; exit 1' SIGINT
source ~/jetauto_ws/devel/setup.bash
roslaunch jetauto_controller jetauto_controller.launch &
PIDlaunch=$!
echo $PIDlaunch
sleep 6

trap 'wait "$PIDrun"' SIGINT
source ~/development_ws/devel/setup.bash
rosrun module1 main.py &
PIDrun=$!
echo $PIDrun
sleep 4

while true; do
    sleep 1
done