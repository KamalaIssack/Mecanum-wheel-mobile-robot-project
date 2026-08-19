"""Starts foxglove_bridge alone, to visualize the running simulation.

Deliberately not bringup.launch.py: that is the real-robot bring-up and also
starts robot_state_publisher (already running from simulation.launch.py) and
joint_state_publisher, which fabricates joint values and would fight the real
/joint_states Gazebo now publishes.

Kept separate so headless runs do not start a websocket server they don't need.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Matches bringup.launch.py so the Foxglove connection string is the same
    # for sim and the real robot.
    bridge_port_arg = DeclareLaunchArgument(
        'bridge_port',
        default_value='8765',
        description='Port for the Foxglove WebSocket bridge',
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': LaunchConfiguration('bridge_port')}],
    )

    return LaunchDescription([
        bridge_port_arg,
        foxglove_bridge_node,
    ])
