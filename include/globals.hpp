#pragma once

// ============================================================================
// PORT / DIRECTION PLACEHOLDERS
//
// Ports below marked "confirmed" are real (given 2026-09-12, oriented with
// the back facing you and the claw facing away). All DIR_* directions are
// now bench-tested (2026-09-18). Flip a DIR_* between 1/-1 to reverse a
// motor instead of hunting through subsystem code.
// ============================================================================

// --- Drivetrain (4-motor skid-steer: 2 left, 2 right) --- confirmed ports + directions
constexpr int DIR_DRIVE_LF = -1;
constexpr int DIR_DRIVE_LB = -1;
constexpr int DIR_DRIVE_RF = 1;
constexpr int DIR_DRIVE_RB = 1;
constexpr int PORT_DRIVE_LF = 19 * DIR_DRIVE_LF;
constexpr int PORT_DRIVE_LB = 11 * DIR_DRIVE_LB;
constexpr int PORT_DRIVE_RF = 16 * DIR_DRIVE_RF;
constexpr int PORT_DRIVE_RB = 20 * DIR_DRIVE_RB;

constexpr int PORT_IMU = 10;  // confirmed port

// Radio is port 21 — plugged in as-is, PROS/VEXnet handle it automatically;
// no code needs to reference it.

// --- Horizontal (X-axis) tracking wheel; Y-axis comes from drive motor encoders ---
// Mounted towards the back of the robot (registered via odom_tracker_back_set in
// main.cpp) and off to the right side, facing right, when looking from behind
// the claw. DIR_ODOM_HORIZONTAL only flips which way the sensor counts up —
// figure that out empirically once it's mounted (push the robot right, the
// reading should increase; flip the sign if it doesn't).
constexpr int DIR_ODOM_HORIZONTAL = -1;  // confirmed by push test 2026-09-18
constexpr int PORT_ODOM_HORIZONTAL = 13 * DIR_ODOM_HORIZONTAL;  // confirmed port
constexpr double ODOM_HORIZONTAL_WHEEL_DIAMETER = 2.0;  // inches, pretty sure — double check against the actual wheel
// Front-back distance from the tracking wheel to the robot's true turning
// center (measure with a tape measure). Since it's a back tracker this is a
// positive number — how far back of center it sits. NOT the left-right offset:
// EZ-Template's back/front trackers only correct for front-back placement: see
// https://ez-robotics.github.io/EZ-Template/tutorials/tuning_tracking_wheel_width
// Being off-center to the right is fine and doesn't need its own parameter here.
constexpr double ODOM_HORIZONTAL_OFFSET = 0.0;

// --- DR4B lift (2 motors, dual-side synced PID) --- confirmed ports
// Previous signs (L=1, R=-1) made the WHOLE lift go down on a positive
// command instead of up — confirmed by testing R1, not by re-derived
// guesswork this time. Both flipped together to invert overall polarity
// while keeping left/right mirrored relative to each other.
constexpr int DIR_LIFT_L = 1;
constexpr int DIR_LIFT_R = -1;
constexpr int PORT_LIFT_L = 8 * DIR_LIFT_L;
constexpr int PORT_LIFT_R = 15 * DIR_LIFT_R;

// --- Claw (1 solenoid) --- confirmed port
constexpr char PORT_CLAW_SOLENOID = 'A';

// --- Toggle spinner + color sensor --- confirmed ports + direction
constexpr int DIR_TOGGLE_SPINNER = -1;
constexpr int PORT_TOGGLE_SPINNER = 17 * DIR_TOGGLE_SPINNER;
constexpr int PORT_TOGGLE_COLOR_SENSOR = 18;
