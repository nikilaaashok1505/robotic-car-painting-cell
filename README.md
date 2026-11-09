# Robotic Car Painting Cell Simulation

- **Name:** Nikilaa Ashok
- **Software used:** ROS 2 Humble, MoveIt 2, RViz2, Python (numpy, scipy, trimesh)
- **Robot selected:** ABB IRB 2400 on a 5.6 m linear rail (7th axis), with a spray gun end-effector

## Overview
A simulated robotic painting cell for a car body (about 4.8 m x 1.9 m x 1.6 m). One industrial robot is placed on one of four rails around the car (near side, far side, front, rear), follows planned painting paths per panel, and the paint it lays down is shown on the car surface in RViz. MoveIt performs trajectory planning and collision checking.

## Repository layout
- `src/painting_cell_moveit_config` - MoveIt config, URDF, SRDF, path generator
- `src/painting_robot_description` - ABB IRB 2400 model and cell description
- `src/painting_trajectory` - trajectory executor and paint visualiser
- `tools/` - helper scripts (`start_helpers.sh`, `set_station.sh`, `set_station.py`, `paint_all.sh`)
- `results/` - run results
- `screenshots/`, `demo_video/` - media

## Results
| Surface | Result |
|---|---|
| Near-side doors | Painted by the arm |
| Far-side doors | Painted by the arm (from the far rail) |
| Front and rear | Painted from the front and rear rails |
| Bonnet | Only partly reachable (about 13% of the path) |
| Roof | Out of reach for this robot; shown as planned coverage only |

## Run
1. Terminal 1 - paths and paint on the car: `bash tools/start_helpers.sh`
2. Terminal 2 - simulation: `ros2 launch painting_cell_moveit_config demo.launch.py`
3. Terminal 3 - paint a panel: `python3 src/painting_trajectory/scripts/painting_executor.py --ros-args -p only:=doors`
4. Move the robot to another rail: `bash tools/set_station.sh near|far|front|rear`, then restart Terminal 2.

Dependencies: ROS 2 Humble, MoveIt 2, `python3-scipy`, `trimesh==3.23.5`, `pycollada`.
