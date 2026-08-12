import math

import rclpy
from rclpy.node import Node

from mecanum_interfaces.msg import WheelSpeeds
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MecanumOdometry(Node):
    def __init__(self):
        super().__init__('mecanum_odometry')

        # Parameters (from the mecanum_odometry block in the YAML).
        self.declare_parameter('wheel_radius', 0.0483)
        self.declare_parameter('lx', 0.1)
        self.declare_parameter('ly', 0.1)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Output topic and TF toggle default to real-robot behavior: this node
        # owns /odom and the odom->base_link transform. Sim comparison overrides
        # both so it runs beside the plugin's bridged /odom and TF.
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('publish_tf', True)

        self.r = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_tf_enabled = self.get_parameter('publish_tf').value

        self.l_sum = self.lx + self.ly

        # Accumulated pose state (this is what makes the node stateful).
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Latest wheel speeds, stored by the subscriber, read by the timer.
        self.w_fl = 0.0
        self.w_fr = 0.0
        self.w_rl = 0.0
        self.w_rr = 0.0

        # Time bookkeeping for real measured dt.
        self.last_time = None  # None flags the first tick

        # Subscriber: store the latest wheel speeds (no math here).
        self.sub = self.create_subscription(
            WheelSpeeds,
            'wheel_speeds',
            self.wheel_speeds_callback,
            10
        )

        # Publisher: the odometry estimate.
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # TF broadcaster only when enabled; otherwise the plugin owns the edge.
        self.tf_broadcaster = None
        if self.publish_tf_enabled:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Timer: fires at publish_rate Hz and does all the integration.
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update)

        self.get_logger().info('mecanum_odometry node started')

    def wheel_speeds_callback(self, msg):
        # Only store. The timer does the work.
        self.w_fl = msg.front_left
        self.w_fr = msg.front_right
        self.w_rl = msg.rear_left
        self.w_rr = msg.rear_right

    def update(self):
        now = self.get_clock().now()

        # First-tick guard: no previous time yet, so just record and wait.
        if self.last_time is None:
            self.last_time = now
            return

        # Real measured dt in seconds.
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Layer 1: forward kinematics (wheel speeds -> body velocity).
        vx = (self.r / 4.0) * (self.w_fl + self.w_fr + self.w_rl + self.w_rr)
        vy = (self.r / 4.0) * (-self.w_fl + self.w_fr + self.w_rl - self.w_rr)
        wz = (self.r / (4.0 * self.l_sum)) * (-self.w_fl + self.w_fr - self.w_rl + self.w_rr)

        # Layer 2: rotate body velocity into the world (odom) frame.
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        vx_world = vx * cos_t - vy * sin_t
        vy_world = vx * sin_t + vy * cos_t

        # Layer 3: integrate to accumulate pose.
        self.x += vx_world * dt
        self.y += vy_world * dt
        self.theta += wz * dt

        # Publish both outputs with a single shared timestamp.
        stamp = now.to_msg()
        self.publish_odometry(stamp, vx, vy, wz)
        if self.publish_tf_enabled:
            self.publish_tf(stamp)

    def publish_odometry(self, stamp, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
