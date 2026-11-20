#!/usr/bin/env python3
"""Keyboard control node for micro-manipulator testing.

This node provides simple keyboard control of the 4 motors using 8 keys:
- q/a : Proximal pan up/down
- w/s : Proximal tilt up/down
- e/d : Distal pan up/down
- r/f : Distal tilt up/down

Each key press changes the motor angle by a configurable step size.

Run it with:
    python3 keyboard_control.py --ros-args -p port:=/dev/ttyACM1
    
The serial bridge must be running separately for this to work.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from motor_angles_msg.msg import MotorAngles

import sys
import termios
import tty

# Servo angular limits -- keep in sync with serial_bridge.py and motor_control_absolute.ino
MIN_ANGLES = (89, 84, 78, 80)
MAX_ANGLES = (100, 95, 100, 94)

# Step size for each key press (in degrees)
DEFAULT_STEP = 1.0


class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")

        self.declare_parameter("step", DEFAULT_STEP)
        self.step = self.get_parameter("step").get_parameter_value().double_value

        # Current motor positions (start at middle of range)
        self.motor_positions = [
            (MIN_ANGLES[i] + MAX_ANGLES[i]) / 2.0
            for i in range(4)
        ]

        # Publisher for motor commands
        self.command_pub = self.create_publisher(
            MotorAngles,
            "micro_module_motor_command",
            10
        )

        # Subscribe to motor states to track actual positions
        self.state_sub = self.create_subscription(
            MotorAngles,
            "micro_module_motor_states",
            self.state_callback,
            10
        )

        self.get_logger().info("Keyboard control node started")
        self.print_instructions()

    def print_instructions(self):
        """Print keyboard control instructions."""
        print("\n" + "="*60)
        print("KEYBOARD CONTROL FOR MICRO-MANIPULATOR")
        print("="*60)
        print(f"Step size: {self.step} degrees")
        print("\nControls:")
        print("  q/a : Proximal pan   (motor 0)  up/down")
        print("  w/s : Proximal tilt  (motor 1)  up/down")
        print("  e/d : Distal pan     (motor 2)  up/down")
        print("  r/f : Distal tilt    (motor 3)  up/down")
        print("\n  +/-  : Increase/decrease step size")
        print("  h    : Print this help message")
        print("  Ctrl+C : Exit")
        print("="*60 + "\n")

    def state_callback(self, msg):
        """Update local position tracking from motor state feedback."""
        self.motor_positions[0] = msg.proximal_pan_angle
        self.motor_positions[1] = msg.proximal_tilt_angle
        self.motor_positions[2] = msg.distal_pan_angle
        self.motor_positions[3] = msg.distal_tilt_angle

    def clamp_and_publish(self):
        """Clamp motor positions to limits and publish command."""
        # Clamp to limits
        for i in range(4):
            self.motor_positions[i] = max(MIN_ANGLES[i], min(MAX_ANGLES[i], self.motor_positions[i]))

        # Create and publish message
        msg = MotorAngles()
        msg.proximal_pan_angle = int(round(self.motor_positions[0]))
        msg.proximal_tilt_angle = int(round(self.motor_positions[1]))
        msg.distal_pan_angle = int(round(self.motor_positions[2]))
        msg.distal_tilt_angle = int(round(self.motor_positions[3]))

        self.command_pub.publish(msg)

        # Display current positions
        print(f"\rPositions: PP={msg.proximal_pan_angle:3d}  "
              f"PT={msg.proximal_tilt_angle:3d}  "
              f"DP={msg.distal_pan_angle:3d}  "
              f"DT={msg.distal_tilt_angle:3d}  ", end="")

    def get_key(self):
        """Get a single keypress from the terminal (non-blocking)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Main keyboard control loop."""
        print("Waiting for motor state feedback...")
        
        # Wait for at least one state message
        while rclpy.ok() and all(p == 0 for p in self.motor_positions):
            rclpy.spin_once(self, timeout_sec=0.1)

        print("\nReady! Use keys to control motors.\n")
        self.clamp_and_publish()

        while rclpy.ok():
            # Process any pending messages
            rclpy.spin_once(self, timeout_sec=0.01)

            # Check for keypress (non-blocking)
            key = self.get_key()

            if key == '\x03':  # Ctrl+C
                print("\nExiting...")
                break
            elif key == 'q':
                self.motor_positions[0] += self.step
            elif key == 'a':
                self.motor_positions[0] -= self.step
            elif key == 'w':
                self.motor_positions[1] += self.step
            elif key == 's':
                self.motor_positions[1] -= self.step
            elif key == 'e':
                self.motor_positions[2] += self.step
            elif key == 'd':
                self.motor_positions[2] -= self.step
            elif key == 'r':
                self.motor_positions[3] += self.step
            elif key == 'f':
                self.motor_positions[3] -= self.step
            elif key == '+':
                self.step = min(10.0, self.step + 0.5)
                print(f"\nStep size increased to {self.step}")
            elif key == '-':
                self.step = max(0.1, self.step - 0.5)
                print(f"\nStep size decreased to {self.step}")
            elif key == 'h':
                self.print_instructions()
            else:
                continue

            self.clamp_and_publish()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()