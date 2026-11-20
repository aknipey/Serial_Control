#!/usr/bin/env python3
"""ROS 2 <-> Arduino serial bridge for the micro-manipulator.

This node is the ROS 2 replacement for rosserial. The old workflow ran a
rosserial node that let the Arduino itself appear as a ROS 1 node. rosserial
does not exist for ROS 2, so instead the Arduino now speaks a tiny plain-text
serial protocol (see ../motor_control/motor_control_absolute/motor_control_absolute.ino)
and this node translates between that protocol and the ROS 2 topics that
`micro_module_control` expects.

Topics
------
Subscribes:  micro_module_motor_command       (motor_angles_msg/MotorAngles)
Publishes:   micro_module_motor_states         (motor_angles_msg/MotorAngles)
             micro_module_motor_min_angles     (motor_angles_msg/MotorAngles, latched-ish)
             micro_module_motor_max_angles      (motor_angles_msg/MotorAngles, latched-ish)

`micro_module_control` will not move the manipulator until it has received the
states, min-angles and max-angles topics, so this bridge is also responsible
for publishing the servo limits. THESE LIMITS MUST MATCH THE FIRMWARE
(the LimitServo min/max constants in motor_control_absolute.ino).

Run it with:
    python3 serial_bridge.py --ros-args -p port:=/dev/ttyACM1
(after sourcing your ROS 2 workspace). It does not need to live in a colcon
package -- it is a standalone rclpy script.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy


import serial  # pip install pyserial

from motor_angles_msg.msg import MotorAngles


# Servo angular limits -- keep in sync with motor_control_absolute.ino.
# Order: (proximal_pan, proximal_tilt, distal_pan, distal_tilt)
MIN_ANGLES = (89, 84, 78, 80)
MAX_ANGLES = (100, 95, 100, 94)

STRING_LENGTH = 12  # four zero-padded 3-digit angles

# Seconds the Mega's bootloader takes after the port opens (it resets on open)
ARDUINO_RESET_TIME = 2.0
# A state arriving this long (s) after the last command was not a reply to it, so the
# Arduino is still in (or back in) its startup handshake. Replies take ~15-30 ms.
UNSOLICITED_STATE_TIME = 0.5


def angles_to_command(pp, pt, dp, dt):
    """Build the 12-digit + newline serial command string."""
    vals = [max(0, min(999, int(round(v)))) for v in (pp, pt, dp, dt)]
    return "{:03d}{:03d}{:03d}{:03d}\n".format(*vals)


def parse_state_string(line):
    """Parse a 12-digit state string into (pp, pt, dp, dt), or None if malformed."""
    line = line.strip()
    if len(line) != STRING_LENGTH or not line.isdigit():
        return None
    return (
        int(line[0:3]),
        int(line[3:6]),
        int(line[6:9]),
        int(line[9:12]),
    )


def make_motor_angles(pp, pt, dp, dt):
    msg = MotorAngles()
    msg.proximal_pan_angle = int(pp)
    msg.proximal_tilt_angle = int(pt)
    msg.distal_pan_angle = int(dp)
    msg.distal_tilt_angle = int(dt)
    return msg


class SerialBridge(Node):
    def __init__(self):
        super().__init__("micro_module_serial_bridge")

        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baud", 9600)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info(f"Opening serial port {port} at {baud} baud...")
        # timeout: blocking reads give up after 1 s so the node stays responsive.
        self.serial = serial.Serial(port, baud, timeout=1.0)
        # Opening the port resets the Arduino (DTR). Wait out its bootloader, then drop
        # whatever was buffered before the reset: answering a stale startup string
        # sends "OK" while the board is still rebooting, it is lost, and the
        # firmware then waits for "OK" forever.
        time.sleep(ARDUINO_RESET_TIME)
        self.serial.reset_input_buffer()

        self.last_state = None          # newest (pp, pt, dp, dt) from the Arduino
        self.last_command_time = 0.0    # time.monotonic() of the last command sent
        self.last_ok_time = 0.0


        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state_pub = self.create_publisher(
            MotorAngles,
            "micro_module_motor_states",
            state_qos
        )
        self.min_pub = self.create_publisher(
            MotorAngles, "micro_module_motor_min_angles", 1
        )
        self.max_pub = self.create_publisher(
            MotorAngles, "micro_module_motor_max_angles", 1
        )

        self.command_sub = self.create_subscription(
            MotorAngles, "micro_module_motor_command", self.command_callback, 1
        )

        self._do_startup_handshake()

        # Re-publish the (static) limits and the last state at 1 Hz so a late-joining
        # control node is guaranteed to receive them regardless of startup ordering.
        # After the handshake the Arduino only reports in reply to a command, and
        # micro_module_control sends nothing until it has a state: without this a
        # node started after the handshake waits forever.
        self.create_timer(1.0, self._republish)
        self.create_timer(0.02, self.read_serial_state)

    def read_serial_state(self):
        while self.serial.in_waiting:
            line = self.serial.readline().decode("ascii", errors="ignore")

            self.get_logger().info(f"Serial received: {line!r}")

            state = parse_state_string(line)

            if state is not None:
                self.get_logger().info(f"Parsed state: {state}")
                self.last_state = state
                self.state_pub.publish(make_motor_angles(*state))
                self._check_still_in_handshake()

    def _check_still_in_handshake(self):
        """After the handshake the Arduino only speaks in reply to a command. A state
        with no command just sent is its startup string again (our "OK" was lost, or
        it reset): acknowledge it again so it leaves the handshake loop."""
        now = time.monotonic()
        if now - self.last_command_time > UNSOLICITED_STATE_TIME and now - self.last_ok_time > 0.5:
            self.serial.write(b"OK\n")
            self.last_ok_time = now
            self.get_logger().warn("Arduino is still sending its startup string - re-sent OK")

    def _do_startup_handshake(self):
        """Read the Arduino's equilibrium string, then acknowledge with 'OK'."""
        self.get_logger().info("Waiting for Arduino startup string...")
        state = None
        while rclpy.ok() and state is None:
            line = self.serial.readline().decode("ascii", errors="ignore")

            self.get_logger().info(f"Arduino returned: {line}")
            state = parse_state_string(line)

        self.serial.write(b"OK\n")
        self.last_ok_time = time.monotonic()
        self.last_state = state
        self.get_logger().info(f"Handshake complete. Equilibrium angles: {state}")
        self.state_pub.publish(make_motor_angles(*state))
        self._publish_limits()

    def _publish_limits(self):
        self.min_pub.publish(make_motor_angles(*MIN_ANGLES))
        self.max_pub.publish(make_motor_angles(*MAX_ANGLES))

    def _republish(self):
        self._publish_limits()
        if self.last_state is not None:
            self.state_pub.publish(make_motor_angles(*self.last_state))

    def command_callback(self, msg):

        command = angles_to_command(
            msg.proximal_pan_angle,
            msg.proximal_tilt_angle,
            msg.distal_pan_angle,
            msg.distal_tilt_angle,
        )

        self.get_logger().info(f"Sending: {command!r}")
        self.serial.write(command.encode("ascii"))
        self.last_command_time = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
