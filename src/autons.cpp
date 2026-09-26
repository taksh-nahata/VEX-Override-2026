#include "main.h"
#include <cmath>

// ============================================================================
// PID / SLEW CONSTANTS
// TODO(tune): carried over from last season's different robot.
// ============================================================================
void default_constants() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.slew_turn_constants_set(5_deg, 50);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.pid_turn_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);
}

// ============================================================================
// AUTONS
// TODO(missing): empty stubs — write once driving/placement are dialed in.
// ============================================================================
void auton_skills() {}
void auton_button_1() {}
void auton_button_2() {}

// ============================================================================
// PID TUNER TEST MOVE
// Runs while the tuner (main.cpp's X/B) is on, to exercise whatever PID is
// selected: drives 24in (Drive PID), then turns 90deg (Turn PID).
// ============================================================================
void tune_test() {
  chassis.pid_drive_set(24_in, 90, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 90, true);
  chassis.pid_wait();
}

// ============================================================================
// DRIVETRAIN CALIBRATION
// For measuring the chassis constructor's gear ratio (main.cpp) and
// ODOM_HORIZONTAL_OFFSET (globals.hpp) — the remaining unmeasured drive
// numbers. Bound to A/LEFT in main.cpp; prints to brain line 5 + controller
// line 2, and every tick logs to /usd/log.csv (sdlog.cpp).
// ============================================================================

// Raw encoder degrees, not inches (inches would bake in the gear ratio
// this measures). 3600 = 10 motor shaft rotations.
constexpr int CALIBRATE_DRIVE_DEGREES = 3600;
constexpr int CALIBRATE_DRIVE_SPEED = 60;  // open-loop, not a PID move

// Tape-measure the real distance driven (M) and report it back:
//   new gear ratio = old gear ratio * (M / drive_in)
// (tracker_in should already be close to M — a big gap there points at
// ODOM_HORIZONTAL_OFFSET or tracker-wheel slop instead.)
void calibrate_straight() {
  chassis.drive_sensor_reset();
  horizontal_tracker.reset();

  chassis.drive_set(CALIBRATE_DRIVE_SPEED, CALIBRATE_DRIVE_SPEED);
  while (std::abs(chassis.drive_sensor_left_raw()) < CALIBRATE_DRIVE_DEGREES &&
         std::abs(chassis.drive_sensor_right_raw()) < CALIBRATE_DRIVE_DEGREES) {
    pros::delay(10);
  }
  chassis.drive_set(0, 0);

  double drive_in = (chassis.drive_sensor_left() + chassis.drive_sensor_right()) / 2.0;
  double tracker_in = horizontal_tracker.get();
  master.print(0, 2, "drv%.1f trk%.1f in", drive_in, tracker_in);
  pros::screen::print(TEXT_MEDIUM, 5, "CALIB straight: drive=%.2fin tracker=%.2fin -- tape-measure real distance",
                       drive_in, tracker_in);
}

// A pure in-place spin has zero real sideways travel, so whatever lateral
// inches the tracker reports is entirely ODOM_HORIZONTAL_OFFSET's fault --
// no human measurement needed, the IMU's rotation count is ground truth.
// More rotations = less noise in the estimate.
constexpr double CALIBRATE_SPIN_ROTATIONS = 8.0;
constexpr int CALIBRATE_SPIN_SPEED = 70;

void calibrate_spin() {
  chassis.drive_imu_reset();
  horizontal_tracker.reset();

  // ez::raw: literal target angle, not "shortest path" (which would see
  // 8*360 as a no-op back to 0).
  chassis.pid_turn_set(CALIBRATE_SPIN_ROTATIONS * 360.0, CALIBRATE_SPIN_SPEED, ez::raw);
  chassis.pid_wait();

  double actual_rotation_deg = chassis.imu.get_rotation();
  double lateral_in = horizontal_tracker.get();
  double radians = actual_rotation_deg * (M_PI / 180.0);
  double offset_estimate = radians != 0 ? lateral_in / radians : 0;

  master.print(0, 2, "off~%.2fin rot%.0f", offset_estimate, actual_rotation_deg);
  pros::screen::print(TEXT_MEDIUM, 5, "CALIB spin: rot=%.1fdeg lateral=%.2fin -> offset~%.3fin", actual_rotation_deg,
                       lateral_in, offset_estimate);
  // Scales linearly with the tracker's true diameter -- if that's
  // corrected later (from calibrate_straight()), rescale this by
  // (new tracker diameter / old) instead of re-running the spin.
}
