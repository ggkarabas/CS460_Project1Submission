#!/usr/bin/env python

#!/usr/bin/env python

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from ament_index_python.packages import get_package_share_directory  # <- Import this
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch


def generate_launch_description():
    # Get the directory of your package and world file
    package_dir = get_package_share_directory('projct1')
    world = LaunchConfiguration('world')
    
    # Webots launcher
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # Add any additional nodes or parameters if necessary (e.g., robot_state_publisher)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # Define the Webots controller or other relevant nodes
    turtlebot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='project1world_gabriellekarabas.wbt',
            description='The world file to load'
        ),
        webots,
        webots._supervisor,
        robot_state_publisher,
        turtlebot_driver,
        
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])

