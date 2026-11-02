#pragma once

// Every port and direction in one place. Flip a DIR_* between 1 and -1 to
// reverse a motor or sensor. Radio is on port 6 (needs no code).

// --- Drivetrain (4-motor skid-steer) ---
constexpr int DIR_DRIVE_LF = -1;
constexpr int DIR_DRIVE_LB = -1;
constexpr int DIR_DRIVE_RF = 1;
constexpr int DIR_DRIVE_RB = 1;
constexpr int PORT_DRIVE_LF = 19 * DIR_DRIVE_LF;
constexpr int PORT_DRIVE_LB = 11 * DIR_DRIVE_LB;
constexpr int PORT_DRIVE_RF = 16 * DIR_DRIVE_RF;
constexpr int PORT_DRIVE_RB = 15 * DIR_DRIVE_RB;

constexpr int PORT_IMU = 10;

// --- Horizontal tracking wheel (sideways); forward/back comes from the drive encoders ---
constexpr int DIR_ODOM_HORIZONTAL = -1;  // push robot right -> reading increases
constexpr int PORT_ODOM_HORIZONTAL = 13 * DIR_ODOM_HORIZONTAL;
constexpr double ODOM_HORIZONTAL_WHEEL_DIAMETER = 2.0;  // inches
// Front/back distance from the wheel to the turning center, measured 2026-09-26
// by spinning 8 times in place and reading the sideways drift (sign as measured).
constexpr double ODOM_HORIZONTAL_OFFSET = -1.97;

// --- DR4B lift: 2 motors on one shaft + rotation sensor on the arm shaft ---
constexpr int DIR_LIFT = 1;
constexpr int PORT_LIFT = 1 * DIR_LIFT;
constexpr int DIR_LIFT_2 = -1;  // opposite of the first motor so they push together
constexpr int PORT_LIFT_2 = 5 * DIR_LIFT_2;
constexpr int DIR_LIFT_ROTATION = -1;
constexpr int PORT_LIFT_ROTATION = 8 * DIR_LIFT_ROTATION;

// --- Claw: 1 solenoid + 1 distance sensor pointing down (lift height comes from it) ---
constexpr char PORT_CLAW_SOLENOID = 'A';
constexpr int PORT_CLAW_DISTANCE = 21;
