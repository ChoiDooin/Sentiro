#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
   
    config_dir = get_package_share_directory('stella_ahrs')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')


    driver_node = LifecycleNode(
        package='stella_ahrs',
        executable='stella_ahrs_node',
        name='stella_ahrs_node', 
        namespace='/', 
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        driver_node,
    ])
