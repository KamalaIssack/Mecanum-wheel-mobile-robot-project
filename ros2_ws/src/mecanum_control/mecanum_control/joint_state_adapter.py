"""
Reshape Gazebo's JointState into mecanum_interfaces/WheelSpeeds.

Lets mecanum_odometry run in simulation unchanged.

Pure structural translation: velocities pass through untouched, so the
per-wheel signs stay identical to the plugin's. On hardware this role is
filled by the Nucleo serial bridge; this node is its sim-only sibling and
pins down the /wheel_speeds contract that bridge must satisfy.
"""

from mecanum_interfaces.msg import WheelSpeeds
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


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
    node = None
    try:
        # Inside the try: a signal during construction must not escape.
        node = JointStateAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # The signal handler has usually shut the context down already;
        # a second shutdown() raises in Jazzy.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
