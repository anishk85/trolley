#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    pkg_path = os.path.join(get_package_share_directory('wd_lift_robot'))
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Launch arguments
    # use_nav2 = LaunchConfiguration('use_nav2')
    # use_slam = LaunchConfiguration('use_slam')
    
    # Robot Description
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])
    
    # RViz config
    # rviz_config_path = os.path.join(pkg_path, 'config', 'robot_view.rviz')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        arguments=[urdf_file]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'wd_lift_robot'],
        output='screen'
    )
    
    # RViz
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path],
    #     parameters=[{'use_sim_time': True}]
    # )
    
    # Controller Manager and Controllers
    load_joint_state_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
        ]
    )
    
    load_diff_drive_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
            ),
        ]
    )
    
    load_lift_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["lift_controller"],
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_nav2',
            default_value='false',
            description='Start Nav2 navigation stack'
        ),
        
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Start SLAM'
        ),
        
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # rviz,
        load_joint_state_controller,
        load_diff_drive_controller,
        load_lift_controller,
    ])