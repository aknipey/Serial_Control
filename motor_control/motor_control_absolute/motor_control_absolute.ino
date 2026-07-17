// motor_control_absolute.ino
//
// ROS 2-era firmware for the micro-manipulator Arduino.
//
// This sketch deliberately uses NOTHING from ROS. rosserial (the <ros.h> /
// ArduinoHardware library used by the old motor_control_ros sketch) only exists
// for ROS 1 and was removed when the project moved to ROS 2 Jazzy. Instead, the
// Arduino now speaks a tiny plain-text serial protocol, and a ROS 2 node on the
// PC (see ../../ros2_bridge/serial_bridge.py) translates between that protocol
// and the ROS 2 topics. The board therefore does not care which ROS version the
// PC runs and works on any Arduino (AVR Uno/Mega/Nano included).
//
// ---------------------------------------------------------------------------
// Serial protocol (9600 baud, newline-terminated)
//
//   Every message is exactly 12 ASCII digits + '\n', four zero-padded 3-digit
//   servo angles in this fixed order (matching motor_angles_msg/MotorAngles):
//
//       [0:3] proximal_pan   [3:6] proximal_tilt   [6:9] distal_pan   [9:12] distal_tilt
//
//   Startup handshake:
//     1. On boot the Arduino moves every servo to its equilibrium angle and
//        repeatedly prints the 12-digit equilibrium string.
//     2. The PC side replies with "OK\n" once it has read a valid string.
//   Steady state:
//     - PC -> Arduino: a 12-digit command = the desired absolute angles.
//     - Arduino moves the servos (clamped to each servo's limits) and replies
//       with a 12-digit string of the actual resulting angles.
// ---------------------------------------------------------------------------

#include "include/LimitServo.hpp"

const int BAUD_RATE = 9600;
const int MOTOR_STATE_STRING_LENGTH = 12;

// Servo signal pins
const int servoDistalTiltPin = 7;
const int servoDistalPanPin = 8;
const int servoProximalPanPin = 9;
const int servoProximalTiltPin = 10;

// Equilibrium (rest) angles
const int servoDistalTiltEqAngle = 88;
const int servoDistalPanEqAngle = 84;
const int servoProximalPanEqAngle = 95;
const int servoProximalTiltEqAngle = 90;

// Lower angular limits
const int servoDistalTiltMinAngle = 80;
const int servoDistalPanMinAngle = 78;
const int servoProximalPanMinAngle = 89;
const int servoProximalTiltMinAngle = 84;

// Upper angular limits
const int servoDistalTiltMaxAngle = 94;
const int servoDistalPanMaxAngle = 100;
const int servoProximalPanMaxAngle = 100;
const int servoProximalTiltMaxAngle = 95;

LimitServo servoProximalPan(
    servoProximalPanPin,
    servoProximalPanEqAngle,
    servoProximalPanMinAngle,
    servoProximalPanMaxAngle
);

LimitServo servoProximalTilt(
    servoProximalTiltPin,
    servoProximalTiltEqAngle,
    servoProximalTiltMinAngle,
    servoProximalTiltMaxAngle
);

LimitServo servoDistalPan(
    servoDistalPanPin,
    servoDistalPanEqAngle,
    servoDistalPanMinAngle,
    servoDistalPanMaxAngle
);

LimitServo servoDistalTilt(
    servoDistalTiltPin,
    servoDistalTiltEqAngle,
    servoDistalTiltMinAngle,
    servoDistalTiltMaxAngle
);

char motorStateString[MOTOR_STATE_STRING_LENGTH + 2];  // 12 digits + '\n' + '\0'

// Fill motorStateString with the current servo angles in protocol order.
void WriteMotorStateString()
{
    sprintf(
        motorStateString,
        "%03d%03d%03d%03d\n",
        servoProximalPan.GetAngle(),
        servoProximalTilt.GetAngle(),
        servoDistalPan.GetAngle(),
        servoDistalTilt.GetAngle()
    );
}

void setup()
{
    Serial.begin(BAUD_RATE);

    servoProximalPan.Attach();
    servoProximalTilt.Attach();
    servoDistalPan.Attach();
    servoDistalTilt.Attach();

    servoProximalPan.MoveToEquilibrium();
    servoProximalTilt.MoveToEquilibrium();
    servoDistalPan.MoveToEquilibrium();
    servoDistalTilt.MoveToEquilibrium();

    while (!Serial)
    {
    }

    // Report equilibrium angles until the PC side acknowledges with "OK".
    bool hasCompletedHandshake = false;

    while (!hasCompletedHandshake)
    {
        WriteMotorStateString();
        Serial.print(motorStateString);

        String response = Serial.readStringUntil('\n');
        hasCompletedHandshake = (response == "OK");
    }
}

void loop()
{
    while (Serial.available())
    {
        String commandString = Serial.readStringUntil('\n');

        if (commandString.length() == MOTOR_STATE_STRING_LENGTH)
        {
            servoProximalPan.Move(commandString.substring(0, 3).toInt());
            servoProximalTilt.Move(commandString.substring(3, 6).toInt());
            servoDistalPan.Move(commandString.substring(6, 9).toInt());
            servoDistalTilt.Move(commandString.substring(9, 12).toInt());

            WriteMotorStateString();
            Serial.print(motorStateString);
        }
    }
}
