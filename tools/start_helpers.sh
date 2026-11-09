#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/automotive_painting_ws/install/setup.bash
cd ~/automotive_painting_ws || exit 1
trap 'kill 0' EXIT
echo ">>> starting planned-path lines"
python3 src/painting_cell_moveit_config/scripts/painting_path_generator.py > /tmp/path_generator.log 2>&1 &
sleep 3
echo ">>> starting paint on the car"
python3 src/painting_trajectory/scripts/paint_surface.py --ros-args -p radius:=0.30 -p paint_color:=0.95,0.40,0.08
