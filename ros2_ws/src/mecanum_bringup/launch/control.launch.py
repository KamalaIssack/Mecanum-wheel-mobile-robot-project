from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_params = PathJoinSubstitution([
        FindPackageShare('mecanum_bringup'),
        'config',
        'mecanum_kinematics.yaml',
    ])

    kinematics_node = Node(
        package='mecanum_control',
        executable='mecanum_kinematics',
        name='mecanum_kinematics',
        output='screen',
        parameters=[control_params],
    )

    odometry_node = Node(
        package='mecanum_control',
        executable='mecanum_odometry',
        name='mecanum_odometry',
        output='screen',
        parameters=[control_params],
    )

    return LaunchDescription([
        kinematics_node,
        odometry_node,
    ])
