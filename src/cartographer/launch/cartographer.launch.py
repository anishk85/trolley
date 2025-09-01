#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    pkg_path = get_package_share_directory('cartographer')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # Configuration files
    cartographer_config_dir_path = os.path.join(pkg_path, 'config')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'cartographer.rviz')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir_path,
            description='Full path to config file to load'
        ),
        
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer.lua',
            description='Name of lua file for cartographer'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz'
        ),
        
        DeclareLaunchArgument(
            'resolution',
            default_value='0.025',
            description='Resolution of a grid cell in the published occupancy grid'
        ),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'
        ),

        # Cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),

        # Cartographer occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'resolution': resolution},
                {'publish_period_sec': publish_period_sec},
                {'width': 2048},
                {'height': 2048},
            ]
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])