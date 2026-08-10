"""Starts ros_gz_bridge from the topic map in config/sim_bridge.yaml.

Kept separate from simulation.launch.py so the gz-transport/ROS 2 boundary
stays a named component that can be restarted without tearing down the
physics world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_bridge = get_package_share_directory('ros_gz_bridge')
    pkg_bringup = get_package_share_directory('mecanum_bringup')

    # Install share dir, not src/: this path only exists after colcon build.
    config_file = os.path.join(pkg_bringup, 'config', 'sim_bridge.yaml')

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_bridge, 'launch', 'ros_gz_bridge.launch.py')
        ),
        launch_arguments={
            'bridge_name': 'sim_bridge',
            'config_file': config_file,
        }.items(),
    )

    return LaunchDescription([bridge])
