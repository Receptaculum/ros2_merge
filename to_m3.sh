#!/bin/sh

cd ~/ros2_merge/src/decision_making_pkg/decision_making_pkg/
ln -s -f m3_motion_planner.py motion_planner.py

cd ~/ros2_merge/src/lidar_perception_pkg/lidar_perception_pkg/
ln -s -f m3_lidar_object_detector.py lidar_object_detector.py

echo "motion_planner <- mission 3"
echo "lidar_object_detector <- mission 3"

cd ~/ros2_merge/
