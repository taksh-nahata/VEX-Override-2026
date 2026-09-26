#include "main.h"
#include <cmath>

// ============================================================================
// PID / SLEW CONSTANTS
// TODO(tune): these are last season's numbers for a different robot, not
// anything we've measured on this one -- placeholders so the drive
// actually moves, not values to trust yet.
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
// TODO(missing): still empty. We wanted driving and placement solid
// before locking in actual scoring routines, so these are stubs for now.
// ============================================================================
void auton_skills() {}
void auton_button_1() {}
void auton_button_2() {}

// ============================================================================
// PID TUNER TEST MOVE
// Something to actually watch happen while the tuner (main.cpp's X/B) is
// on: drives 24in to exercise Drive PID, then turns 90deg to exercise
// Turn PID, so we're not just staring at numbers change with no move to
// judge them against.
// ============================================================================
void tune_test() {
  chassis.pid_drive_set(24_in, 90, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 90, true);
  chassis.pid_wait();
}

// ============================================================================
// DRIVETRAIN CALIBRATION
// Two test moves for the drivetrain numbers we still don't actually know:
// the gear ratio in main.cpp's chassis constructor, and
// ODOM_HORIZONTAL_OFFSET in globals.hpp. Bound to A/LEFT in main.cpp;
// results print to brain line 5 and controller line 2, and every tick
// also lands in /usd/log.csv (sdlog.cpp) in case we need to look closer
// afterward.
// ============================================================================

// We drive in raw encoder degrees here, not inches -- inches would
// already assume the gear ratio we're trying to measure, which would
// make the whole test circular. 3600 is 10 motor shaft rotations, enough
// distance to tape-measure precisely.
constexpr int CALIBRATE_DRIVE_DEGREES = 3600;
constexpr int CALIBRATE_DRIVE_SPEED = 60;  // open-loop, not a PID move -- we're not trusting distance yet

// Tape-measure the real distance driven (call it M) and tell us the
// number. From there:
//   new gear ratio = old gear ratio * (M / drive_in)
// If tracker_in is also off from M by a lot more than drive_in is, that
// points at ODOM_HORIZONTAL_OFFSET or slop in the tracker wheel's mount
// instead of the gear ratio.
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

// This one we figured out doesn't need a human measurement at all: a pure
// in-place spin has zero real sideways travel, so any lateral inches the
// tracker reports during one can only be explained by
// ODOM_HORIZONTAL_OFFSET being wrong. The IMU's own rotation count gives
// us ground truth for how far we actually spun, so we can solve for the
// offset ourselves. More rotations averages out more of the noise.
constexpr double CALIBRATE_SPIN_ROTATIONS = 8.0;
// Dropped from 70 to 35 (2026-09-26) as a one-off test -- the robot was
// landing a little off-angle after a spin, and we want to see whether
// that's the IMU's gyro getting less accurate at higher spin speeds
// (would improve at 35) or just Turn PID's exit tolerance being loose
// (wouldn't change with speed, and is a separate tuning job anyway). Put
// this back to 70 once that's answered.
constexpr int CALIBRATE_SPIN_SPEED = 35;

void calibrate_spin() {
  chassis.drive_imu_reset();
  chassis.odom_xyt_set(0, 0, 0);  // clean slate so odom_x/odom_y below start at true 0
  horizontal_tracker.reset();

  // ez::raw asks for the literal target angle instead of the shortest
  // path there -- with "shortest path" behavior, 8 full rotations would
  // just look like 0 and the robot wouldn't move at all.
  chassis.pid_turn_set(CALIBRATE_SPIN_ROTATIONS * 360.0, CALIBRATE_SPIN_SPEED, ez::raw);
  chassis.pid_wait();

  double actual_rotation_deg = chassis.imu.get_rotation();
  double lateral_in = horizontal_tracker.get();
  double radians = actual_rotation_deg * (M_PI / 180.0);
  double offset_estimate = radians != 0 ? lateral_in / radians : 0;

  // Two different numbers here, for two different moments: offset_estimate
  // (from lateral_in, the raw wheel reading) is what we solved
  // ODOM_HORIZONTAL_OFFSET from the first time this test ran. odom_x/odom_y
  // are the chassis's own corrected position estimate, which is what
  // actually uses that constant -- run this test again after applying a
  // fix and check THESE stay near 0, not lateral_in again (that number
  // doesn't change just because we updated the constant).
  master.print(0, 2, "odX%.2f odY%.2f", chassis.odom_x_get(), chassis.odom_y_get());
  pros::screen::print(TEXT_MEDIUM, 5,
                       "CALIB spin: rot=%.1fdeg raw_lateral=%.2fin off_est~%.3fin  odom_x=%.2f odom_y=%.2f",
                       actual_rotation_deg, lateral_in, offset_estimate, chassis.odom_x_get(), chassis.odom_y_get());
}
