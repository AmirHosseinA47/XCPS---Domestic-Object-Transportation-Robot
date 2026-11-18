#!/bin/bash
# Build everything and run milestone 3 (objects, head camera, pick and place, behaviour tree).
# Run inside WSL:  bash /mnt/e/Projects/XCPS_Kitchen_Gazebo/run_milestone3.sh [set|clean]
set -e
SCENARIO=${1:-set}
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")"
python3 src/kitchen_gazebo/scripts/make_gazebo_urdfs.py
python3 src/kitchen_gazebo/scripts/make_bridge_config.py
python3 src/kitchen_gazebo/scripts/make_nav2_params.py
colcon build
source install/setup.bash
ros2 launch kitchen_gazebo kitchen_tasks.launch.py > ~/kitchen_tasks.log 2>&1 &
LAUNCH=$!
trap 'kill $LAUNCH 2>/dev/null' EXIT
echo "simulation starting (log: ~/kitchen_tasks.log); the behaviour tree waits for the robot skills ..."
ros2 run kitchen_bt kitchen_bt_runner --scenario "$SCENARIO"
echo "press Enter to stop the simulation"; read -r
