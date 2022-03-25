#!/bin/bash
cd ~/version3_ws
source devel/setup.bash 
roslaunch nav_code nav_all.launch &
sleep 5s
cd ~/version3_ws/src/competition_code/scripts
python3 auto_system.py
echo "ok"
