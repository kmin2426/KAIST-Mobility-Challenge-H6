#3. accel cav

source /opt/ros/foxy/setup.bash
source ~/Mobility_Challenge_Simulator/install/setup.bash
export ROS_DOMAIN_ID=100

cd ~/Mobility_Challenge_Simulator/tool

# (중요) 혹시 남은 pub 있으면 죽이기
pkill -f "ros2 topic pub"
pkill -f "cav_waypoint_follower"

python3 cav_waypoint_follower.py

