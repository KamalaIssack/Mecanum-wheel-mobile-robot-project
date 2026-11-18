"""
Real-robot bring-up.

Starts the hardware stack: kinematics, the Nucleo serial bridge, odometry,
robot_state_publisher and foxglove_bridge.

joint_state_publisher is deliberately absent: nucleo_bridge publishes the real
/joint_states, and a second publisher would fight it with invented values.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Each node reads only the YAML block keyed by its name=. A mismatch is
    # ignored silently and leaves the code defaults in force.
    kinematics_params = PathJoinSubstitution([
        FindPackageShare('mecanum_bringup'),
        'config',
        'mecanum_kinematics.yaml',
    ])

    bridge_params = PathJoinSubstitution([
        FindPackageShare('mecanum_bringup'),
        'config',
        'nucleo_bridge.yaml',
    ])

    default_model_path = PathJoinSubstitution([
        FindPackageShare('mecanum_description'),
        'urdf',
        'mecanum_robot.urdf.xacro',
    ])

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to the robot xacro file',
    )

    # Matches bringup.launch.py and sim_viz.launch.py so the Foxglove
    # connection string is the same for sim and the real robot.
    bridge_port_arg = DeclareLaunchArgument(
        'bridge_port',
        default_value='8765',
        description='Port for the Foxglove WebSocket bridge',
    )

    # value_type=str stops launch_ros from parsing the URDF XML as YAML.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    # Renamed on this node only, so /wheel_speeds carries nothing but the
    # bridge's measured speeds and odometry integrates real wheel motion.
    kinematics_node = Node(
        package='mecanum_control',
        executable='mecanum_kinematics',
        name='mecanum_kinematics',
        output='screen',
        parameters=[kinematics_params],
        remappings=[('wheel_speeds', 'wheel_speeds_cmd')],
    )

    # Without the bridge, odometry would keep integrating the last measured
    # speeds and /odom would drift unnoticed, so its exit ends the launch.
    # Not respawned: the bridge arms the motors on start-up, and restarting
    # after a fault must be a human decision.
    bridge_node = Node(
        package='mecanum_control',
        executable='nucleo_bridge',
        name='nucleo_bridge',
        output='screen',
        parameters=[bridge_params],
        on_exit=Shutdown(reason='nucleo_bridge exited, stopping the stack'),
    )

    odometry_node = Node(
        package='mecanum_control',
        executable='mecanum_odometry',
        name='mecanum_odometry',
        output='screen',
        parameters=[kinematics_params],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': LaunchConfiguration('bridge_port')}],
    )

    return LaunchDescription([
        model_arg,
        bridge_port_arg,
        kinematics_node,
        bridge_node,
        odometry_node,
        robot_state_publisher_node,
        foxglove_bridge_node,
    ])
