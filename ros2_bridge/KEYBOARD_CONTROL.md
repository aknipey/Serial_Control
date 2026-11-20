# Keyboard Control for Micro-Manipulator

A simple keyboard interface for testing the micro-manipulator motors without the haptic device.

## Prerequisites

1. **Serial bridge must be running:**
   ```bash
   python3 serial_bridge.py --ros-args -p port:=/dev/ttyACM1
   ```

2. **Source your ROS 2 workspace:**
   ```bash
   source install/setup.bash
   ```

## Running the Keyboard Control

```bash
python3 keyboard_control.py
```

Or with a custom step size:
```bash
python3 keyboard_control.py --ros-args -p step:=2.0
```

## Controls

| Key | Motor | Direction |
|-----|-------|-----------|
| q   | Proximal Pan   | Increase |
| a   | Proximal Pan   | Decrease |
| w   | Proximal Tilt  | Increase |
| s   | Proximal Tilt  | Decrease |
| e   | Distal Pan     | Increase |
| d   | Distal Pan     | Decrease |
| r   | Distal Tilt    | Increase |
| f   | Distal Tilt    | Decrease |
| +   | - | Increase step size (0.5° increments, max 10°) |
| -   | - | Decrease step size (0.5° decrements, min 0.1°) |
| h   | - | Show help message |
| Ctrl+C | - | Exit |

## Motor Limits

The motors are automatically clamped to these limits (in degrees):
- Proximal Pan: 89-100
- Proximal Tilt: 84-95
- Distal Pan: 78-100
- Distal Tilt: 80-94

## Troubleshooting

**"Waiting for motor state feedback..." forever:**
- Make sure the serial bridge is running
- Check that the Arduino is connected to the correct port
- Verify the serial bridge is receiving data from the Arduino

**Motors not moving:**
- Check that the serial bridge is sending commands (look for "Sending:" in its output)
- Verify the Arduino is receiving and parsing commands correctly
- Make sure motor limits in keyboard_control.py match those in serial_bridge.py and the Arduino firmware

**Keys not responding:**
- Make sure the terminal window is in focus
- Try pressing keys while looking at the terminal
- Check that you're not in a text editor that intercepts keypresses