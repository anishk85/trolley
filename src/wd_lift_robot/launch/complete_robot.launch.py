#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Package Directories
    pkg_path = get_package_share_directory('wd_lift_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_nav2 = LaunchConfiguration('use_nav2')
    use_slam = LaunchConfiguration('use_slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    
    # Robot Description - ONLY YOUR ROBOT
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])
    
    # Configuration files
    nav2_params_path = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    slam_toolbox_config = os.path.join(pkg_path, 'config', 'slam_toolbox_params.yaml')
    rviz_config_path = os.path.join(pkg_path, 'config', 'robot_view.rviz')
    controllers_config = os.path.join(pkg_path, 'config', 'controllers.yaml')
    
    # World file - Use the small_house.world from your package
    world_file = os.path.join(pkg_path, 'worlds', 'small_house.world')
    
    # Launch Gazebo server with CLEAN environment (no TurtleBot3)
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', world_file, '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': '', 'TURTLEBOT3_MODEL': ''}  # Clear TurtleBot3 env vars
    )

    # Launch Gazebo client
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Robot State Publisher - ONLY YOUR ROBOT
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
    )
    
    # Spawn ONLY YOUR ROBOT - wd_lift_robot
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description', 
                    '-entity', 'wd_lift_robot',  # Unique entity name
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '0.05',
                    '-R', '0',
                    '-P', '0',
                    '-Y', '0'
                ],
                output='screen'
            )
        ]
    )
    
    # Controller Manager
    controller_manager = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[controllers_config, {'use_sim_time': use_sim_time}],
                output="screen",
            )
        ]
    )
    
    # Load controllers
    load_joint_state_controller = TimerAction(
        period=7.0,
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
        period=9.0,
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
        period=11.0,
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
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': use_sim_time}]
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
                parameters=[slam_toolbox_config, {'use_sim_time': use_sim_time}],
                condition=IfCondition(use_slam)
            )
        ]
    )
    
    # Nav2 stack
    nav2_bringup = TimerAction(
        period=16.0,
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
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
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
        
        DeclareLaunchArgument(
            'x_pose',
            default_value='-2.0',
            description='Initial x position of robot'
        ),
        
        DeclareLaunchArgument(
            'y_pose',
            default_value='-0.5',
            description='Initial y position of robot'
        ),
        
        # Gazebo components
        gzserver_cmd,
        gzclient_cmd,
        
        # Robot components - ONLY YOUR ROBOT
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        load_joint_state_controller,
        load_diff_drive_controller,
        load_lift_controller,
        
        # Visualization and navigation
        rviz,
        slam_toolbox,
        nav2_bringup,
    ])

# added comment for testing workflows
