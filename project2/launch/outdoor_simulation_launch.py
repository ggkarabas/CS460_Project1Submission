#!/usr/bin/env python

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():
    # Get the directory of your package and world file
    package_dir = get_package_share_directory('project2')
    world = LaunchConfiguration('world')
    
    # Webots launcher
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # Define any additional nodes, e.g., for robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # Add Webots driver or other nodes if needed
    turtlebot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='outdoorworld_gabriellekarabas.wbt',
            description='The world file to load'
        ),
        webots,
        webots._supervisor,
        robot_state_publisher,
        turtlebot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        ),
    ])

