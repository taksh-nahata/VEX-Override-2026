#pragma once

// ============================================================================
// PORT / DIRECTION PLACEHOLDERS
// Flip a DIR_* between 1/-1 to reverse a motor/sensor instead of hunting
// through subsystem code. Ports oriented with the back facing you, claw
// facing away.
// ============================================================================

// --- Drivetrain (4-motor skid-steer: 2 left, 2 right) --- confirmed
constexpr int DIR_DRIVE_LF = -1;
constexpr int DIR_DRIVE_LB = -1;
constexpr int DIR_DRIVE_RF = 1;
constexpr int DIR_DRIVE_RB = 1;
constexpr int PORT_DRIVE_LF = 19 * DIR_DRIVE_LF;
constexpr int PORT_DRIVE_LB = 11 * DIR_DRIVE_LB;
constexpr int PORT_DRIVE_RF = 16 * DIR_DRIVE_RF;
constexpr int PORT_DRIVE_RB = 15 * DIR_DRIVE_RB;

constexpr int PORT_IMU = 10;

// Radio is port 21 — plugged in as-is, PROS/VEXnet handle it automatically.

// --- Horizontal (X-axis) tracking wheel; Y-axis comes from drive encoders ---
// Mounted toward the back, off to the right (registered via
// odom_tracker_back_set in main.cpp).
constexpr int DIR_ODOM_HORIZONTAL = -1;  // confirmed: push robot right -> reading increases
constexpr int PORT_ODOM_HORIZONTAL = 13 * DIR_ODOM_HORIZONTAL;
constexpr double ODOM_HORIZONTAL_WHEEL_DIAMETER = 2.0;  // inches — confirmed

// Front-back distance from the tracking wheel to the robot's true turning
// center (positive = behind center). NOT left-right offset — EZ-Template's
// back/front trackers only correct for front-back placement. TODO(verify):
// never measured, still 0.0 — see calibrate_spin() in autons.cpp.
constexpr double ODOM_HORIZONTAL_OFFSET = 0.0;

// --- DR4B lift (1 motor, second four-bar, 1:6 external reduction) --- confirmed
// Rotation sensor mounted on the 72T (four-bar) shaft, past the reduction
// — reads true arm angle instead of trusting the motor's encoder through
// the gear mesh. See lift.cpp for how it's used.
constexpr int DIR_LIFT = 1;
constexpr int PORT_LIFT = 1 * DIR_LIFT;

constexpr int DIR_LIFT_ROTATION = -1;
constexpr int PORT_LIFT_ROTATION = 8 * DIR_LIFT_ROTATION;

// --- Claw (1 solenoid) --- confirmed
constexpr char PORT_CLAW_SOLENOID = 'A';

// --- Toggle spinner + color sensor --- confirmed
constexpr int DIR_TOGGLE_SPINNER = -1;
constexpr int PORT_TOGGLE_SPINNER = 17 * DIR_TOGGLE_SPINNER;
constexpr int PORT_TOGGLE_COLOR_SENSOR = 18;
