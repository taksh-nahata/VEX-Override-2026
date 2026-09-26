#pragma once

// ============================================================================
// PORTS AND DIRECTIONS
// We keep every port and spin direction in one place so fixing a wiring
// mistake is a one-line change here instead of a hunt through subsystem
// code. Flip a DIR_* between 1 and -1 to reverse a motor or sensor. Ports
// assume the back of the robot is facing you and the claw is facing away.
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

// Distance from the tracking wheel to the robot's true turning center.
// Not the left-right offset — EZ-Template's back/front trackers only
// correct for front-back placement, and being off-center side to side
// doesn't need a parameter here.
//
// Measured 2026-09-26 with calibrate_spin() (autons.cpp): spun the robot
// 8 full rotations in place with this at 0.0 and read off how much
// sideways drift the tracking wheel picked up from a move that should
// have had none. We plug that number in directly, sign and all, rather
// than assuming which sign means "in front" vs. "behind" — we don't have
// visibility into which convention EZ-Template's odometry math actually
// uses internally, and getting that guess wrong would be worse than just
// trusting the number the test produced. Re-running calibrate_spin() now
// checks chassis.odom_x_get()/odom_y_get() (the corrected position, not
// the raw wheel reading) to confirm this actually zeroed the drift out.
constexpr double ODOM_HORIZONTAL_OFFSET = -1.97;

// --- DR4B lift (1 motor, second four-bar, 1:6 external reduction) --- confirmed
// The rotation sensor sits on the 72T (four-bar) shaft, past the
// reduction, so it tells us the arm's real angle instead of us trusting
// the motor's own encoder through the gear mesh. See lift.cpp for the
// full reasoning and how each one gets used.
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
