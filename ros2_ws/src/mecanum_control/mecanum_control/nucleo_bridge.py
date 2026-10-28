"""
Serial bridge between ROS 2 and the Nucleo motor controller (teaching version).

ROS to Nucleo: the latest /wheel_speeds_cmd is re-sent as a C line every
100 ms (the heartbeat that feeds the firmware's 300 ms watchdog). A command
older than cmd_timeout is replaced by zeros, so a dead upstream node stops
the robot instead of the heartbeat hiding it. The change per heartbeat is
limited by max_accel.

Nucleo to ROS: every V line (measured wheel speeds, 50 Hz) is published as
WheelSpeeds on /wheel_speeds, the same contract joint_state_adapter fills in
simulation, so mecanum_odometry runs unchanged on the real robot.

Start-up: E,0, then K and I, each confirmed by its reply, then E,1. The node
refuses to arm if any reply is missing. Exit: C,0,0,0,0 then E,0.
"""

import time

from mecanum_interfaces.msg import WheelSpeeds
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial


class NucleoBridge(Node):
    def __init__(self):
        super().__init__('nucleo_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('kp', 10.0)
        self.declare_parameter('ki', 100.0)
        self.declare_parameter('heartbeat_period', 0.1)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_accel', 20.0)
        self.declare_parameter('max_wheel_speed', 41.9)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.period = self.get_parameter('heartbeat_period').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_step = self.get_parameter('max_accel').value * self.period
        self.max_speed = self.get_parameter('max_wheel_speed').value

        # timeout=0 makes read() return at once with whatever has arrived.
        self.ser = serial.Serial(port, baud, timeout=0)
        self.rx_buf = b''

        # Latest command and when it arrived; None means nothing received yet.
        self.target = [0.0, 0.0, 0.0, 0.0]
        self.target_time = None
        # What was actually sent last, the starting point for the ramp.
        self.sent = [0.0, 0.0, 0.0, 0.0]

        self.armed = False
        try:
            self.handshake()
        except RuntimeError:
            self.stop_board()  # never leave the board armed after a failure
            raise
        self.armed = True

        self.pub = self.create_publisher(WheelSpeeds, 'wheel_speeds', 10)
        self.sub = self.create_subscription(
            WheelSpeeds, 'wheel_speeds_cmd', self.on_cmd, 10
        )
        # Read often (V and D arrive at 50 Hz each); send on the heartbeat.
        self.read_timer = self.create_timer(0.01, self.read_serial)
        self.send_timer = self.create_timer(self.period, self.send_heartbeat)

        self.get_logger().info('nucleo_bridge armed on %s' % port)

    # ----- serial helpers -------------------------------------------------

    def write_line(self, text):
        self.ser.write((text + '\n').encode('ascii'))

    def read_lines(self):
        """Return complete lines received so far; keep any partial line."""
        self.rx_buf += self.ser.read(self.ser.in_waiting or 1)
        lines = []
        while b'\n' in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b'\n', 1)
            lines.append(raw.decode('ascii', 'replace').strip())
        return lines

    def wait_for(self, expected, timeout=0.5):
        """Wait until a line equal to `expected` arrives, or give up."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if expected in self.read_lines():
                return True
        return False

    # ----- start and stop ---------------------------------------------------

    def handshake(self):
        # Same port settle as the bench tools before the first write.
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self.write_line('E,0')
        if not self.wait_for('S,DISARMED'):
            self.write_line('E,0')  # one retry
            if not self.wait_for('S,DISARMED'):
                raise RuntimeError('no S,DISARMED: is the Nucleo connected?')

        # Gains boot at 0 and are only accepted while disarmed.
        for tag, value in (('K', self.kp), ('I', self.ki)):
            self.write_line('%s,%.2f' % (tag, value))
            if not self.wait_for('A,%s,%.2f' % (tag, value)):
                raise RuntimeError('gain %s,%.2f not acknowledged' % (tag, value))

        self.write_line('E,1')
        if not self.wait_for('S,ARMED'):
            raise RuntimeError('no S,ARMED reply to E,1')

    def stop_board(self):
        """Zero the wheels and disarm. Safe to call more than once."""
        if self.ser.is_open:
            self.write_line('C,0,0,0,0')
            self.write_line('E,0')
            time.sleep(0.05)
            self.ser.close()

    # ----- ROS callbacks ----------------------------------------------------

    def on_cmd(self, msg):
        # Only store; the heartbeat timer decides what is sent and when.
        self.target = [msg.front_left, msg.front_right,
                       msg.rear_left, msg.rear_right]
        self.target_time = time.monotonic()

    def send_heartbeat(self):
        fresh = (self.target_time is not None and
                 time.monotonic() - self.target_time < self.cmd_timeout)
        goal = self.target if fresh else [0.0, 0.0, 0.0, 0.0]

        for i in range(4):
            g = max(-self.max_speed, min(self.max_speed, goal[i]))
            step = max(-self.max_step, min(self.max_step, g - self.sent[i]))
            self.sent[i] += step

        self.write_line('C,%.3f,%.3f,%.3f,%.3f' % tuple(self.sent))

    def read_serial(self):
        for line in self.read_lines():
            if line.startswith('V,'):
                self.publish_measured(line)
            elif line == 'S,TIMEOUT':
                self.armed = False
                self.get_logger().error(
                    'Nucleo watchdog fired (S,TIMEOUT): motors disarmed. '
                    'Restart the bridge to re-arm.'
                )
            # A,... and D,... lines are not used in this version.

    def publish_measured(self, line):
        parts = line.split(',')
        if len(parts) != 5:
            return
        try:
            fl, fr, rl, rr = (float(x) for x in parts[1:])
        except ValueError:
            return  # a garbled line; the next one arrives in 20 ms
        out = WheelSpeeds()
        out.front_left = fl
        out.front_right = fr
        out.rear_left = rl
        out.rear_right = rr
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NucleoBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as err:
        print('nucleo_bridge: %s' % err)
    finally:
        if node is not None:
            node.stop_board()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
