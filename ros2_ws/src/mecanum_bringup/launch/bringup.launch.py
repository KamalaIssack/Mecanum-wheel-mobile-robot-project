from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    bridge_port_arg = DeclareLaunchArgument(
        'bridge_port',
        default_value='8765',
        description='Port for the Foxglove WebSocket bridge',
    )

    # Expand xacro at launch time; value_type=str keeps the XML intact.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
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
        robot_state_publisher_node,
        joint_state_publisher_node,
        foxglove_bridge_node,
    ])
