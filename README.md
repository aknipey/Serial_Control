# Serial_Control — micro-manipulator firmware & ROS 2 serial bridge

Firmware and serial glue for the Arduino-driven continuum micro-manipulator of
the Macro-Micro surgical robot. The main control package (ROS 2) lives here:
https://github.com/LMBScott/ls_thesis/

> **Note:** This repository was originally written for **ROS 1** using
> `rosserial` (the `<ros.h>` Arduino library), which let the Arduino act as a
> ROS 1 node directly. `rosserial` does **not** exist for ROS 2. The project has
> been migrated to **ROS 2 Jazzy**, so the Arduino now speaks a plain-text serial
> protocol and a small ROS 2 node on the PC (`ros2_bridge/serial_bridge.py`)
> translates between that protocol and the ROS 2 topics. See *How it fits
> together* below.

## Why this is a separate repository

The PC runs the `ls_thesis` ROS 2 package, which does all the maths and decides
*what angle each servo should be at*. But the servos are wired to an **Arduino**,
a completely separate microcontroller with its own processor, toolchain and
language (Arduino C++ `.ino`, flashed with the Arduino IDE — **not** built by
colcon). This repo holds that **firmware** plus the bridge that connects it to
ROS 2. Keeping it separate means:

- The ROS 2 package stays hardware-agnostic — it just publishes desired angles.
- The firmware can be flashed and tested on its own (see `src/main.cpp`, a
  keyboard tester that drives the servos with no ROS at all).
- The two halves are built with different tools and target different processors.

## How it fits together

```
  3D Systems Touch ─┐
                    │   ROS 2 topics
   UR5e driver ─────┤                          plain-text serial (USB)
                    ▼                                    │
        micro_module_control  ──(MotorAngles)──►  serial_bridge.py  ──►  Arduino
        (in ls_thesis, ROS 2)  ◄─(MotorAngles)──  (this repo)        ◄──  (this repo,
                                                                          motor_control_absolute.ino)
```

- `micro_module_control` publishes desired servo angles on
  `micro_module_motor_command` and listens on `micro_module_motor_states`,
  `micro_module_motor_min_angles`, `micro_module_motor_max_angles`.
- `serial_bridge.py` converts those ROS 2 messages to/from the 12-character
  serial protocol and publishes the servo limits the control node waits for.
- `motor_control_absolute.ino` runs on the Arduino, moves the four servos and
  reports their angles back.

### Serial protocol

9600 baud, newline-terminated. Every message is four zero-padded 3-digit servo
angles (12 chars + `\n`) in the order
`proximal_pan, proximal_tilt, distal_pan, distal_tilt` (matching
`motor_angles_msg/MotorAngles`). On boot the Arduino prints its equilibrium
angles until the PC replies `OK\n`; thereafter the PC sends a 12-digit command
and the Arduino replies with the resulting angles.

## Contents

| Path | What it is |
| --- | --- |
| `motor_control/motor_control_absolute/` | **Current** Arduino firmware (plain serial, ROS 2-compatible). Flash this. |
| `ros2_bridge/serial_bridge.py` | ROS 2 node bridging serial ↔ ROS 2 topics (the rosserial replacement). |
| `motor_control/motor_control_ros/` | **Legacy** ROS 1 rosserial firmware. Kept for reference; does not work with ROS 2. |
| `motor_control/motor_control_relative.ino` | Old relative-motion sketch (reference only). |
| `src/main.cpp`, `lib/`, `include/`, `makefile` | Standalone ncurses keyboard tester (no ROS) for bench-testing the servos. |

## Installation & use (Ubuntu 24.04 + ROS 2 Jazzy)

### 1. Flash the Arduino firmware
1. Install the Arduino IDE.
2. Install the **Servo** library (Arduino IDE → Library Manager) — the only
   dependency now that rosserial is gone.
3. Open `motor_control/motor_control_absolute/motor_control_absolute.ino` and
   upload it to the board.

### 2. Run the ROS 2 bridge
1. Build/​source the `ls_thesis` workspace so `motor_angles_msg` is available.
2. Install pyserial: `pip install pyserial` (or `sudo apt install python3-serial`).
3. Give yourself access to the serial device and find which port it is:
   ```
   ls /dev/ttyACM*           # e.g. /dev/ttyACM1
   sudo usermod -aG dialout $USER   # then log out/in once
   ```
4. Run the bridge:
   ```
   source ~/ros2_ws/install/setup.bash
   python3 ros2_bridge/serial_bridge.py --ros-args -p port:=/dev/ttyACM1
   ```
5. In another terminal, start the control node:
   ```
   ros2 run ls_thesis micro_module_control
   ```

### Bench-testing the servos without ROS (optional)
`src/main.cpp` drives the servos directly over serial with the keyboard.
Build with `make` (needs `libncurses-dev`) and run `./bin/main`.
