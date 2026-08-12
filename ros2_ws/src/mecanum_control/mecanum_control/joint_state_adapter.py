"""Reshapes Gazebo's sensor_msgs/JointState into mecanum_interfaces/WheelSpeeds
so mecanum_odometry can run in simulation unchanged.

Pure structural translation: velocities pass through untouched, so the
per-wheel signs stay identical to the plugin's. On hardware this role is
filled by the Nucleo serial bridge; this node is its sim-only sibling and
pins down the /wheel_speeds contract that bridge must satisfy.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from mecanum_interfaces.msg import WheelSpeeds


class JointStateAdapter(Node):
    def __init__(self):
        super().__init__('joint_state_adapter')

        self.wheel_joints = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        self.pub = self.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10
        )

    def on_joint_states(self, msg: JointState):
        # Look up by name, not array index: JointState ordering is not
        # contractual, and the serial bridge will likewise parse by field.
        velocity_by_name = dict(zip(msg.name, msg.velocity))

        missing = [j for j in self.wheel_joints if j not in velocity_by_name]
        if missing:
            self.get_logger().warn(
                f'JointState missing {missing}; skipping this message'
            )
            return

        out = WheelSpeeds()
        out.front_left = velocity_by_name['front_left_wheel_joint']
        out.front_right = velocity_by_name['front_right_wheel_joint']
        out.rear_left = velocity_by_name['rear_left_wheel_joint']
        out.rear_right = velocity_by_name['rear_right_wheel_joint']

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
