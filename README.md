## Scope
This branch contains my individual implementation for the following tasks:

- **task 1-1**: Waypoint-based control for a single CAV  
- **task 2**: Control and overtaking logic in mixed traffic with HVs and a CAV
- **task 3**: Multi-CAV control in complex road structures
  
## How to Run

### 1. Simulator Execution
```bash
cd ~/Desktop/Mobility_Challenge_Simulator

source /opt/ros/foxy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=100

ros2 launch simulator_launch simulator_launch.py
```

### 2. Control Code Execution (H6 Repository)
```bash
# ROS2 Foxy
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=100

# Simulator workspace
source ~/Desktop/Mobility_Challenge_Simulator/install/setup.bash

# H6 workspace build
cd ~/Desktop/KAIST-Mobility-Challenge-H6
colcon build --symlink-install
source install/setup.bash

# Run decision node
cd ~/Desktop/KAIST-Mobility-Challenge-H6/task3
python3 final_task3.py
```

## Foler Structure

```text
dev/dongmin/
├── path_visualize.py
├── task1-1/
│   └── task1_1.py
├── task2/
│   ├── path/
│   │   ├── path_2_1.json
│   │   ├── path_2_2.json
│   │   ├── path_2_3.json
│   │   ├── path_2_4.json
│   │   ├── lane_change.json
│   │   └── not_lane_change.json
│   ├── visualization_proj.py
│   ├── projection.py
│   ├── decision_fsm.py
│   └── controller.py
└── task3/
    ├── path/
    │   ├── path3_1.json
    │   ├── path3_2.json
    │   ├── path3_3.json
    │   ├── path3_4.json
    │   │   └─ CAV reference paths for Task 3
    │   │
    │   ├── path3_1_zone.csv
    │   ├── path3_2_zone.csv
    │   ├── path3_3_zone.csv
    │   ├── path3_4_zone.csv
    │   │   └─ Stop / trigger zones for CAV decision logic
    │   │
    │   ├── path3_1_out_zone.csv
    │   ├── path3_2_out_zone.csv
    │   │   └─ Exit & reset zones after leaving the control area
    │   │
    │   ├── path_hv.csv
    │   │   └─ Full HV reference trajectory
    │   │
    │   ├── path_hv_2_1.csv
    │   ├── path_hv_2_2.csv
    │   ├── path_hv_3_1.csv
    │   ├── path_hv_3_2.csv
    │       └─ HV trigger / gate paths used for interaction logic
    │
    ├── final_task3.py               # Main execution file
    ├── cross.py                     # Intersection-specific logic
    ├── round.py                     # Roundabout-specific logic
    ├── vis.py                       # Debug & visualization utilities
    └── dynamic_speed/               # Speed profiles & tuning

