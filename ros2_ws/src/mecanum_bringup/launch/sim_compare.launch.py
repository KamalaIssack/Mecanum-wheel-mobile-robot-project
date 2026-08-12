"""Path B: runs joint_state_adapter and mecanum_odometry so the project's own
forward-kinematics code produces /odom_computed alongside the plugin's bridged
/odom, for side-by-side validation.

mecanum_odometry is overridden into comparison mode: it publishes on
/odom_computed and does not broadcast TF, since the bridge already carries the
plugin's odom->base_link edge. Kept separate from the sim and bridge launches
so this stack restarts independently; assumes both are already running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('mecanum_bringup')
    params = os.path.join(pkg_bringup, 'config', 'mecanum_kinematics.yaml')

    adapter = Node(
        package='mecanum_control',
        executable='joint_state_adapter',
        output='screen',
    )

    # YAML first for geometry, then the two overrides that flip comparison
    # mode on (later entries win).
    odometry = Node(
        package='mecanum_control',
        executable='mecanum_odometry',
        output='screen',
        parameters=[
            params,
            {'odom_topic': 'odom_computed'},
            {'publish_tf': False},
        ],
    )

    return LaunchDescription([adapter, odometry]) 
