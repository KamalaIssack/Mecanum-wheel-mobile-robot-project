from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    kinematics_params = PathJoinSubstitution([
        FindPackageShare('mecanum_bringup'),
        'config',
        'mecanum_kinematics.yaml',
    ])

    kinematics_node = Node(
        package='mecanum_control',
        executable='mecanum_kinematics',
        name='mecanum_kinematics',
        output='screen',
        parameters=[kinematics_params],
    )

    return LaunchDescription([
        kinematics_node,
    ])
