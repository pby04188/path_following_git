#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('path_following'),
            'config',
            'path_following_deepracer.yaml'))
    
    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(
            get_package_share_directory('path_following'),
            'rviz',
            'pathfollowing.rviz'))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of path file'
        ),

        Node(
            package='path_following',
            executable='pid_planner',
            name='pid_planner',
            parameters=[param_dir],
            output='screen',
            remappings=[('/ctrl_cmd', '/deepracer/cmd_vel')]),
        
        
        Node(
            package='path_following',
            executable='pose_stamped_subscriber',
            name='pose_stamped_subscriber',
            output='screen'),
])
        
         
