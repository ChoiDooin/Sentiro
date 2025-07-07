#!/usr/bin/env python3

# Author: Bishop Pearson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
  # ros_namespace = LaunchConfiguration('ros_namespace')
  
  omo_r1_mcu_parameter = LaunchConfiguration(
    'omo_r1_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('main_bringup'),
      'params/omo_r1_mcu.yaml'
    )
  )
  lidar_parameter = LaunchConfiguration(
    'lidar_parameter',
    default=os.path.join(
      get_package_share_directory('ydlidar_ros2_driver'),
      'params/ydlidar.yaml'
    )
  )

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  omo_r1_description_dir = LaunchConfiguration(
    'omo_r1_description_dir',
    default=os.path.join(
      get_package_share_directory('main_description'),
      'launch'
    )
  )
  
  lidar_dir = LaunchConfiguration(
    'lidar_dir',
    default=os.path.join(
      get_package_share_directory('ydlidar_ros2_driver'),
      'launch'
    )
  )

  return LaunchDescription([
    # DeclareLaunchArgument(
    #         'ros_namespace',
    #         default_value=os.environ['ROS_NAMESPACE'],
    #         description='Namespace for the robot'),
    
    DeclareLaunchArgument(
      'omo_r1_mcu_parameter',
      # namespace=ros_namespace,
      default_value=omo_r1_mcu_parameter
    ),

    DeclareLaunchArgument(
      'omo_r1_lidar_parameter',
      # namespace=ros_namespace,
      default_value=lidar_parameter
    ),
    

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1_mcu.launch.py']),
      launch_arguments={'omo_r1_mcu_parameter': omo_r1_mcu_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([lidar_dir, '/ydlidar_launch.py']),
      launch_arguments={'omo_r1_lidar_parameter': lidar_parameter}.items()
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([omo_r1_description_dir, '/omo_r1_state_publisher.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
      
    ),
  ])
