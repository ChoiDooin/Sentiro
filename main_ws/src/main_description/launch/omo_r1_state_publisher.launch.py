#!/usr/bin/env python3

# Author: Bishop Pearson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'omo_r1.urdf'

    urdf = os.path.join(
        get_package_share_directory('main_description'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # robot_state_publisher 노드 설정
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ]
    )
    # joint_state_publisher 노드 설정
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    tf2_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
     )
    tf2_odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_static_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    # tf2_base_footprint_to_base_link = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='tf_static_base_footprint_to_base_link',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    # )


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        tf2_map_to_odom,
        tf2_odom_to_base_footprint,
        # tf2_base_footprint_to_base_link,
    ])
