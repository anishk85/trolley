#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Package Directories
    pkg_path = get_package_share_directory('wd_lift_robot')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_nav2 = LaunchConfiguration('use_nav2')
    use_slam = LaunchConfiguration('use_slam')
    
    # Robot Description
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])
    
    # Configuration files
    nav2_params_path = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    slam_toolbox_config = os.path.join(pkg_path, 'config', 'slam_toolbox_params.yaml')
    rviz_config_path = os.path.join(pkg_path, 'config', 'robot_view.rviz')
    controllers_config = os.path.join(pkg_path, 'config', 'controllers.yaml')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'false'}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )
    
    # Spawn Robot
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'wd_lift_robot'],
                output='screen'
            )
        ]
    )
    
    # Controller Manager
    controller_manager = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_config, {'use_sim_time': True}],
                output="screen",
            )
        ]
    )
    
    # Load controllers
    load_joint_state_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen"
            ),
        ]
    )
    
    load_diff_drive_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            ),
        ]
    )
    
    load_lift_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["lift_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            ),
        ]
    )
    
    # RViz
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # SLAM Toolbox
    slam_toolbox = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_toolbox_config, {'use_sim_time': True}],
                condition=IfCondition(use_slam)
            )
        ]
    )
    
    # Nav2 stack
    nav2_bringup = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_path
                }.items(),
                condition=IfCondition(use_nav2)
            )
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
        spawn_entity,
        controller_manager,
        load_joint_state_controller,
        load_diff_drive_controller,
        load_lift_controller,
        rviz,
        slam_toolbox,
        nav2_bringup,
    ])