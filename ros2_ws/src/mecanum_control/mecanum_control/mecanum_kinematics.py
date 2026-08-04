import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mecanum_interfaces.msg import WheelSpeeds





class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')

        # Declare parameters with defaults. The YAML overrides these at launch.
        self.declare_parameter('wheel_radius', 0.0483)
        self.declare_parameter('lx', 0.1)
        self.declare_parameter('ly', 0.1)
        self.declare_parameter('max_wheel_speed', 41.9)

        # Read them once into plain Python floats.
        self.r = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value

        # Precompute the lever-arm sum; it never changes.
        self.l_sum = self.lx + self.ly

        # Publisher: the wheel-speed command we compute.
        self.wheel_pub = self.create_publisher(WheelSpeeds, 'wheel_speeds', 10)

        # Subscriber: incoming body-velocity commands.
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('mecanum_kinematics node started')


    def cmd_vel_callback(self, msg):
        # Extract body velocities (REP-103: +x fwd, +y left, +z CCW yaw).
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Inverse kinematics. Each row is one physical wheel.
        # Locked matrix: FL,RR rollers '\'  FR,RL rollers '/'
        w_fl = (vx - vy - self.l_sum * wz) / self.r
        w_fr = (vx + vy + self.l_sum * wz) / self.r
        w_rl = (vx + vy - self.l_sum * wz) / self.r
        w_rr = (vx - vy + self.l_sum * wz) / self.r

        # Saturate each wheel to the motor's physical ceiling.
        w_fl = self._clamp(w_fl)
        w_fr = self._clamp(w_fr)
        w_rl = self._clamp(w_rl)
        w_rr = self._clamp(w_rr)

        # Publish.
        out = WheelSpeeds()
        out.front_left = w_fl
        out.front_right = w_fr
        out.rear_left = w_rl
        out.rear_right = w_rr
        self.wheel_pub.publish(out)


    def _clamp(self, value):
        # Saturate to +/- max_wheel_speed without changing sign.
        if value > self.max_wheel_speed:
            return self.max_wheel_speed
        if value < -self.max_wheel_speed:
            return -self.max_wheel_speed
        return value




def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

