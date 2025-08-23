#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    navigation_pkg_path = get_package_share_directory('navigation')
    robot_pkg_path = get_package_share_directory('wd_lift_robot')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Configuration files
    default_map_file = os.path.join(navigation_pkg_path, 'maps', 'house_map.yaml')
    default_params_file = os.path.join(navigation_pkg_path, 'config', 'trolley.yaml')
    rviz_config_file = os.path.join(navigation_pkg_path, 'rviz', 'nav2.rviz')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map file to load'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to param file to load'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        # Launch robot first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_pkg_path, 'launch', 'complete_robot.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x_pose': '-2.0',
                'y_pose': '-0.5'
            }.items(),
        ),

        # Map Server
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'yaml_filename': map_yaml_file
                    }]
                )
            ]
        ),

        # AMCL
        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[params_file, {'use_sim_time': use_sim_time}]
                )
            ]
        ),

        # Controller Server
        TimerAction(
            period=17.0,
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    output='screen',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    remappings=[('/cmd_vel', '/cmd_vel_nav')]  # Output to intermediate topic
                )
            ]
        ),

        # Planner Server
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[params_file, {'use_sim_time': use_sim_time}]
                )
            ]
        ),

        # Behavior Server
        TimerAction(
            period=19.0,
            actions=[
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[params_file, {'use_sim_time': use_sim_time}]
                )
            ]
        ),

        # BT Navigator
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[params_file, {'use_sim_time': use_sim_time}]
                )
            ]
        ),

        # Waypoint Follower
        TimerAction(
            period=21.0,
            actions=[
                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    parameters=[params_file, {'use_sim_time': use_sim_time}]
                )
            ]
        ),

        TimerAction(
            period=22.0,
            actions=[
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    parameters=[params_file, {'use_sim_time': use_sim_time}],
                    remappings=[
                        ('/cmd_vel', '/cmd_vel_nav'),                      # Input from controller (Twist)
                        ('/cmd_vel_smoothed', '/diff_drive_controller/cmd_vel')  # Output to robot (TwistStamped)
                    ]
                )
            ]
        ),

        # Lifecycle Manager for localization
        TimerAction(
            period=23.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': ['map_server', 'amcl']
                    }]
                )
            ]
        ),

        # Lifecycle Manager for navigation
        TimerAction(
            period=24.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': [
                            'controller_server',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator',
                            'waypoint_follower',
                            'velocity_smoother'
                        ]
                    }]
                )
            ]
        ),

        # RViz
        TimerAction(
            period=25.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        ),
    ])