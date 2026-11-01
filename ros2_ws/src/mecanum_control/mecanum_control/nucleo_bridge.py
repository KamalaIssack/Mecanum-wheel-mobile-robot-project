"""
Serial bridge between ROS 2 and the Nucleo motor controller.

ROS to Nucleo: the latest /wheel_speeds_cmd is re-sent as a C line every
heartbeat_period (the heartbeat that feeds the firmware's 300 ms watchdog).
A command older than cmd_timeout is replaced by zeros, so a dead upstream
node stops the robot instead of the heartbeat hiding it. The change per
heartbeat is limited by max_accel.

Nucleo to ROS: every V line (measured wheel speeds, 50 Hz) is published on
/wheel_speeds, the same contract joint_state_adapter fills in simulation, so
mecanum_odometry runs unchanged on the real robot. The board state from S
lines is published on /nucleo_status.

Start-up: newline, E,0, then K and I, each confirmed by its reply, then E,1.
The node refuses to arm if any reply is missing. Exit: C,0,0,0,0 then E,0.
"""

import math
import time

from mecanum_interfaces.msg import WheelSpeeds
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import serial
from std_msgs.msg import String

ZEROS = [0.0, 0.0, 0.0, 0.0]
BOARD_STATES = ('ARMED', 'DISARMED', 'TIMEOUT')


class BridgeError(RuntimeError):
    """A failure that ends the bridge with one readable message."""

    # main() catches only this type. Anything else is a bug and keeps its
    # traceback.


def parse_four(line, tag):
    """Return the four numbers of a '<tag>,a,b,c,d' line, or None if malformed."""
    parts = line.split(',')
    # Five fields also rejects the gain acks A,K,<kp> and A,I,<ki>, so they
    # are never counted as velocity acks.
    if len(parts) != 5 or parts[0] != tag:
        return None
    try:
        values = tuple(float(p) for p in parts[1:])
    except ValueError:
        return None
    # float() accepts 'nan' and 'inf'; neither is ever a real speed.
    if not all(math.isfinite(v) for v in values):
        return None
    return values


def parse_status(line):
    """Return 'ARMED', 'DISARMED' or 'TIMEOUT' for an S line, else None."""
    if line.startswith('S,') and line[2:] in BOARD_STATES:
        return line[2:]
    return None


def fresh_goal(target, target_time, now, timeout):
    """Return the target if it is younger than timeout seconds, else zeros."""
    if target_time is None or now - target_time >= timeout:
        return list(ZEROS)
    return list(target)


def ramp_toward(sent, goal, max_step, max_speed):
    """Move each wheel from sent toward goal by at most max_step per call."""
    out = []
    for s, g in zip(sent, goal):
        # The firmware drops a whole C line if any value exceeds 42 rad/s.
        g = max(-max_speed, min(max_speed, g))
        step = max(-max_step, min(max_step, g - s))
        out.append(s + step)
    return out


class NucleoBridge(Node):
    """ROS 2 node that owns the Nucleo serial port."""

    def __init__(self):
        super().__init__('nucleo_bridge')

        # Defaults mirror config/nucleo_bridge.yaml in mecanum_bringup, so a
        # bare 'ros2 run' behaves the same as the launched node.
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('kp', 10.0)
        self.declare_parameter('ki', 100.0)
        self.declare_parameter('heartbeat_period', 0.1)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('ack_timeout', 0.5)
        self.declare_parameter('max_accel', 20.0)
        self.declare_parameter('max_wheel_speed', 41.9)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.period = self.get_parameter('heartbeat_period').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.ack_timeout = self.get_parameter('ack_timeout').value
        self.max_step = self.get_parameter('max_accel').value * self.period
        self.max_speed = self.get_parameter('max_wheel_speed').value

        # The state is published only on change, so late subscribers such as
        # Foxglove need TRANSIENT_LOCAL to receive the current one.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'nucleo_status', latched)
        self.state = None

        # SerialException is a subclass of OSError.
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
        except OSError as err:
            raise BridgeError('cannot open %s: %s' % (port, err))
        self.rx_buf = b''

        self.target = list(ZEROS)
        self.target_time = None
        self.sent = list(ZEROS)  # last value sent: the ramp's starting point

        try:
            self.handshake()
        except BridgeError:
            self.stop_board()  # never leave the board armed after a failure
            raise

        self.set_state('ARMED')
        self.last_ack_time = time.monotonic()
        self.ack_warned = False

        self.pub = self.create_publisher(WheelSpeeds, 'wheel_speeds', 10)
        self.sub = self.create_subscription(
            WheelSpeeds, 'wheel_speeds_cmd', self.on_cmd, 10
        )
        # V and D lines arrive at 50 Hz each, so read far more often than that.
        self.read_timer = self.create_timer(0.01, self.read_serial)
        self.send_timer = self.create_timer(self.period, self.send_heartbeat)

        self.get_logger().info('nucleo_bridge armed on %s' % port)

    # ----- serial helpers -------------------------------------------------

    def write_line(self, text):
        """Send one line, turning a dead port into a BridgeError."""
        try:
            self.ser.write((text + '\n').encode('ascii'))
        except OSError as err:
            raise BridgeError('serial link lost (%s): is the USB cable connected?' % err)

    def read_lines(self):
        """Return complete lines received so far; keep any partial line."""
        try:
            # in_waiting itself raises on a vanished device.
            self.rx_buf += self.ser.read(self.ser.in_waiting or 1)
        except OSError as err:
            raise BridgeError('serial link lost (%s): is the USB cable connected?' % err)
        lines = []
        while b'\n' in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b'\n', 1)
            lines.append(raw.decode('ascii', 'replace').strip())
        return lines

    def wait_for(self, expected, timeout=0.5):
        """Wait until a line equal to expected arrives; True if it did."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if expected in self.read_lines():
                return True
        return False

    # ----- start and stop ---------------------------------------------------

    def handshake(self):
        """Disarm, set and confirm both gains, then arm."""
        time.sleep(0.2)  # port settle, as in the bench tools
        self.ser.reset_input_buffer()

        # Ends any partial line left in the firmware's buffer, which would
        # otherwise swallow the first E,0. The firmware ignores empty lines.
        self.write_line('')

        self.write_line('E,0')
        if not self.wait_for('S,DISARMED'):
            self.get_logger().warning('first E,0 unanswered, retrying once')
            self.write_line('E,0')
            if not self.wait_for('S,DISARMED'):
                raise BridgeError('no S,DISARMED: is the Nucleo connected?')

        # Gains boot at 0 and are accepted only while disarmed.
        for tag, value in (('K', self.kp), ('I', self.ki)):
            self.write_line('%s,%.2f' % (tag, value))
            if not self.wait_for('A,%s,%.2f' % (tag, value)):
                raise BridgeError('gain %s,%.2f not acknowledged' % (tag, value))

        self.write_line('E,1')
        if not self.wait_for('S,ARMED'):
            raise BridgeError('no S,ARMED reply to E,1')

    def stop_board(self):
        """Zero the wheels and disarm. Safe to call more than once."""
        if not self.ser.is_open:
            return
        try:
            self.write_line('C,0,0,0,0')
            self.write_line('E,0')
            time.sleep(0.05)  # let the bytes leave before the port closes
        except BridgeError:
            # Link already gone: the firmware watchdog disarms the board
            # 300 ms after the last command.
            pass
        finally:
            self.ser.close()

    def set_state(self, state):
        """Publish the board state on /nucleo_status when it changes."""
        if state == self.state:
            return
        self.state = state
        self.status_pub.publish(String(data=state))

    # ----- ROS callbacks ----------------------------------------------------

    def on_cmd(self, msg):
        """Store the newest wheel command; the heartbeat decides what is sent."""
        self.target = [msg.front_left, msg.front_right,
                       msg.rear_left, msg.rear_right]
        self.target_time = time.monotonic()

    def send_heartbeat(self):
        """Send one ramped C line, and warn if the board has stopped answering."""
        now = time.monotonic()
        goal = fresh_goal(self.target, self.target_time, now, self.cmd_timeout)
        self.sent = ramp_toward(self.sent, goal, self.max_step, self.max_speed)
        self.write_line('C,%.3f,%.3f,%.3f,%.3f' % tuple(self.sent))

        # The firmware acks every valid C line, armed or not, so silence means
        # a garbled link, wrong firmware or a hung board. Warn once per spell.
        if not self.ack_warned and now - self.last_ack_time > self.ack_timeout:
            self.ack_warned = True
            self.get_logger().warning(
                'no velocity ack for %.1f s: is the board still answering?'
                % (now - self.last_ack_time)
            )

    def read_serial(self):
        """Route each complete line from the board to its handler."""
        for line in self.read_lines():
            if line.startswith('V,'):
                self.publish_measured(line)
            elif line.startswith('A,'):
                self.note_ack(line)
            elif line.startswith('S,'):
                self.on_status(line)

    def publish_measured(self, line):
        """Publish one V line as a WheelSpeeds message on /wheel_speeds."""
        values = parse_four(line, 'V')
        if values is None:
            return  # garbled; the next line arrives in 20 ms
        out = WheelSpeeds()
        out.front_left, out.front_right, out.rear_left, out.rear_right = values
        self.pub.publish(out)

    def note_ack(self, line):
        """Restart the ack clock on a velocity ack."""
        if parse_four(line, 'A') is None:
            return
        self.last_ack_time = time.monotonic()
        if self.ack_warned:
            self.ack_warned = False
            self.get_logger().info('velocity acks resumed')

    def on_status(self, line):
        """Handle an S line: publish it, and log anything unexpected."""
        state = parse_status(line)
        if state is None:
            return
        if state == 'TIMEOUT':
            self.get_logger().error(
                'Nucleo watchdog fired (S,TIMEOUT): motors disarmed. '
                'Restart the bridge to re-arm.'
            )
        elif state == 'DISARMED' and self.state == 'ARMED':
            # Only this node sends E,0, and only on exit.
            self.get_logger().warning('board disarmed by another sender')
        self.set_state(state)


def main(args=None):
    """Run the bridge; return 0 on a normal stop, 1 on a bridge failure."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = NucleoBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except BridgeError as err:
        # Logger by name: the node may never have been created.
        rclpy.logging.get_logger('nucleo_bridge').error(str(err))
        exit_code = 1
    finally:
        if node is not None:
            node.stop_board()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
