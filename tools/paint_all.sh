#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/automotive_painting_ws || exit 1
source install/setup.bash

run_station () {
  st="$1"; shift
  echo "=================== STATION: $st ==================="
  python3 ~/set_station.py "$st" src/painting_cell_moveit_config/config || return
  colcon build --packages-select painting_cell_moveit_config 2>&1 | tail -1
  source install/setup.bash
  ros2 launch painting_cell_moveit_config demo.launch.py > /tmp/demo_$st.log 2>&1 &
  LP=$!
  for i in $(seq 1 60); do
    ros2 action list 2>/dev/null | grep -q move_action && break
    sleep 3
  done
  sleep 15
  for p in "$@"; do
    echo "--- painting $p from the $st rail ---"
    python3 src/painting_trajectory/scripts/painting_executor.py --ros-args -p only:=$p -p min_fraction:=0.05 2>&1 | grep -E "fraction|SUCCEEDED|failed|ERROR|approach="
  done
  kill -INT $LP 2>/dev/null; sleep 4
  pkill -f rviz2; pkill -f move_group; pkill -f ros2_control_node; pkill -f robot_state_publisher; pkill -f spawner
  sleep 5
}

ros2 topic pub --once /paint_reset std_msgs/msg/Empty "{}" > /dev/null 2>&1
run_station near  doors bonnet roof
run_station far   doors_far bonnet roof
run_station front front bonnet
run_station rear  rear_body roof

python3 ~/set_station.py near src/painting_cell_moveit_config/config
colcon build --packages-select painting_cell_moveit_config 2>&1 | tail -1
echo "ALL STATIONS DONE"
