#!/bin/sh

cd ~/ros2_merge/src/decision_making_pkg/decision_making_pkg/
ln -s -f m2_motion_planner.py motion_planner.py

cd ~/ros2_merge/src/lidar_perception_pkg/lidar_perception_pkg/
ln -s -f m2_lidar_object_detector.py lidar_object_detector.py

cd ~/ros2_merge/src/debug_pkg/debug_pkg/
ln -s -f m2_lidar_debugger.py lidar_debugger.py

echo "motion_planner <- mission 2"
echo "lidar_object_detector <- mission 2"
echo "lidar_debugger.py <- mission 2"

cd ~/ros2_merge/
