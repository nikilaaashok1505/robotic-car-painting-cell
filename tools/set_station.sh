#!/bin/bash
# usage: bash ~/set_station.sh near|far|front|rear
source /opt/ros/humble/setup.bash
cd ~/automotive_painting_ws || exit 1
python3 ~/set_station.py "$1" src/painting_cell_moveit_config/config || exit 1
colcon build --packages-select painting_cell_moveit_config 2>&1 | tail -3
echo "DONE: station = $1  -> now (re)start Tab 1"
