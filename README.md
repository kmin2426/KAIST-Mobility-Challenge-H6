## Scope
This branch contains my individual implementation for the following tasks:

- **task 1-1**: Waypoint-based control for a single CAV  
- **task 2**: Control and overtaking logic in mixed traffic with HVs and a CAV
  
## How to Run

### 1. Simulator Execution
```bash
cd ~/Desktop/Mobility_Challenge_Simulator

source /opt/ros/foxy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=100

ros2 launch simulator_launch simulator_launch.py
```

### 2. Control Code Execution (Simulator Repository)
```bash
source /opt/ros/foxy/setup.bash
source ~/Desktop/Mobility_Challenge_Simulator/install/setup.bash

export ROS_DOMAIN_ID=100

cd ~/Desktop/Mobility_Challenge_Simulator/src/central_control/task3
python3 task2_1.py
```

### 3. Control Code Execution (H6 Repository)
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
python3 decision.py
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
