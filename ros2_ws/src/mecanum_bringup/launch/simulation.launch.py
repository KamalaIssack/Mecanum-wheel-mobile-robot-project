"""Headless Gazebo bring-up: starts the server, publishes the robot
description, and spawns the model into the world.

The ros_gz_bridge is intentionally kept in a separate launch file so the
gz-transport/ROS 2 boundary stays explicit rather than buried here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('mecanum_description')

    default_model = os.path.join(
        pkg_description, 'urdf', 'mecanum_robot.urdf.xacro'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to the robot xacro',
    )

    # value_type=str keeps the expanded xacro as a string; without it launch
    # treats the XML as a filename.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    # Server-only: no GUI process, required in the Codespaces dev environment.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -s empty.sdf'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # -z offset spawns the robot just above the ground so wheel collisions
    # settle onto contact instead of resolving an initial interpenetration.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecanum_robot',
            '-z', '0.05',
        ],
    )

    return LaunchDescription([
        model_arg,
        gazebo,
        robot_state_publisher,
        spawn,
    ])
