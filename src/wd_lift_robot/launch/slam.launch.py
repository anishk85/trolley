#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import conditions
from launch import substitutions

def generate_launch_description():
    # Get the launch directory
    pkg_path = get_package_share_directory('wd_lift_robot')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_method = LaunchConfiguration('slam_method')
    
    # SLAM Toolbox configuration
    slam_toolbox_config = os.path.join(pkg_path, 'config', 'slam_toolbox_params.yaml')
    
    # Cartographer configuration
    cartographer_config_dir = os.path.join(pkg_path, 'config')
    cartographer_config_basename = 'cartographer.lua'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        
        DeclareLaunchArgument(
            'slam_method',
            default_value='slam_toolbox',
            description='SLAM method to use: slam_toolbox or cartographer'
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_toolbox_config, {'use_sim_time': use_sim_time}],
            condition=conditions.IfCondition(
                substitutions.PythonExpression([
                    '"', slam_method, '" == "slam_toolbox"'
                ])
            )
        ),

        # Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                      '-configuration_basename', cartographer_config_basename],
            condition=conditions.IfCondition(
                substitutions.PythonExpression([
                    '"', slam_method, '" == "cartographer"'
                ])
            )
        ),

        # Cartographer Occupancy Grid Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            condition=conditions.IfCondition(
                substitutions.PythonExpression([
                    '"', slam_method, '" == "cartographer"'
                ])
            )
        ),
    ])