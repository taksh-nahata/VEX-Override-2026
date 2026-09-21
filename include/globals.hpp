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
constexpr int PORT_DRIVE_RB = 15 * DIR_DRIVE_RB;  // moved from 20 to 15 2026-09-20 to make room for the lift motor

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
constexpr double ODOM_HORIZONTAL_WHEEL_DIAMETER = 2.0;  // inches — confirmed 2026-09-20
// Front-back distance from the tracking wheel to the robot's true turning
// center. Since it's a back tracker this is a positive number — how far
// back of center it sits. NOT the left-right offset: EZ-Template's
// back/front trackers only correct for front-back placement: see
// https://ez-robotics.github.io/EZ-Template/tutorials/tuning_tracking_wheel_width
// Being off-center to the right is fine and doesn't need its own parameter here.
// TODO(verify): measure with a tape measure — never done, still 0.0.
constexpr double ODOM_HORIZONTAL_OFFSET = 0.0;

// --- DR4B lift (1 motor, mounted on the second four-bar) --- confirmed port
// Rebuilt 2026-09-20 from the old 2-motor dual-side design — that design's
// left/right gear mismatch (one tooth off, found 2026-09-18) kept causing
// current spikes/skipping no matter how sync correction was tuned, since
// it was a fixed mechanical disagreement no encoder-based correction could
// reach. Moving to a single motor removes the mismatch entirely instead of
// working around it: confirmed on the bench, no more skipping.
//
// 12T on the motor, 72T on the second four-bar's shaft — 1:6 external
// reduction. A rotation sensor is mounted on the 72T shaft (confirmed port
// 2026-09-20) for direct arm-angle feedback past that gear mesh instead of
// trusting the motor's own encoder through it — see lift.cpp for why, and
// for how it's actually used (position, not current/driving — that's
// still the motor).
//
// TODO(verify): both DIR_LIFT and DIR_LIFT_ROTATION are guesses (DIR_LIFT
// matches the old DIR_LIFT_L) — confirm each with R1 like every other
// actuator's direction this project, don't trust either blind just because
// it's a default. They're independent: the rotation sensor could easily
// read backwards from the motor even if the motor's sign is already right.
constexpr int DIR_LIFT = 1;
constexpr int PORT_LIFT = 20 * DIR_LIFT;

constexpr int DIR_LIFT_ROTATION = 1;
constexpr int PORT_LIFT_ROTATION = 12 * DIR_LIFT_ROTATION;

// --- Claw (1 solenoid) --- confirmed port
constexpr char PORT_CLAW_SOLENOID = 'A';

// --- Toggle spinner + color sensor --- confirmed ports + direction
constexpr int DIR_TOGGLE_SPINNER = -1;
constexpr int PORT_TOGGLE_SPINNER = 17 * DIR_TOGGLE_SPINNER;
constexpr int PORT_TOGGLE_COLOR_SENSOR = 18;
