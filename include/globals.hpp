#pragma once

// all ports in one place, flip a DIR_ to reverse something. radio is on 6

// drivetrain
constexpr int DIR_DRIVE_LF = -1;
constexpr int DIR_DRIVE_LB = -1;
constexpr int DIR_DRIVE_RF = 1;
constexpr int DIR_DRIVE_RB = 1;
constexpr int PORT_DRIVE_LF = 19 * DIR_DRIVE_LF;
constexpr int PORT_DRIVE_LB = 11 * DIR_DRIVE_LB;
constexpr int PORT_DRIVE_RF = 16 * DIR_DRIVE_RF;
constexpr int PORT_DRIVE_RB = 15 * DIR_DRIVE_RB;

constexpr int PORT_IMU = 10;

// sideways tracking wheel
constexpr int DIR_ODOM_HORIZONTAL = -1;  // pushing the robot right should increase it
constexpr int PORT_ODOM_HORIZONTAL = 13 * DIR_ODOM_HORIZONTAL;
constexpr double ODOM_HORIZONTAL_WHEEL_DIAMETER = 2.0;  // inches
// offset from the center of the robot, measured by spinning it 8 times
constexpr double ODOM_HORIZONTAL_OFFSET = -1.97;

// lift
constexpr int DIR_LIFT = 1;
constexpr int PORT_LIFT = 1 * DIR_LIFT;
constexpr int DIR_LIFT_2 = -1;  // opposite of the first motor
constexpr int PORT_LIFT_2 = 5 * DIR_LIFT_2;
constexpr int DIR_LIFT_ROTATION = -1;
constexpr int PORT_LIFT_ROTATION = 8 * DIR_LIFT_ROTATION;

// claw. CLAW_DISTANCE points down for lift height, CLAW_PIN_DISTANCE faces the pins
constexpr char PORT_CLAW_SOLENOID = 'A';
constexpr int PORT_CLAW_DISTANCE = 21;
constexpr int PORT_CLAW_PIN_DISTANCE = 9;
