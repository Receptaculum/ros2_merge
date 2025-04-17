#!/usr/bin/env python3

import os
import subprocess
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
            
    return LaunchDescription([                   
        Node(
            package='camera_perception_pkg', 
            executable='image_publisher',
            output='screen'
        ),       

        Node(
            package='camera_perception_pkg', 
            executable='yolov8',
            output='screen'
        ),

        Node(
            package='camera_perception_pkg', 
            executable='traffic_light_detector',
            output='screen'
        ),

        Node(
            package='camera_perception_pkg', 
            executable='car_info_extractor',
            output='screen'
        ),

        Node(
            package='camera_perception_pkg', 
            executable='lane_info_extractor',
            output='screen'
        ),
       
        Node(
            package='camera_perception_pkg', 
            executable='depth_extractor',
            output='screen'
        ),

        Node(
            package='decision_making_pkg', 
            executable='motion_planner',
            output='screen'
        ),

        Node(
            package='debug_pkg', 
            executable='data_debugger',
            output='screen'
        ),     
    ])
