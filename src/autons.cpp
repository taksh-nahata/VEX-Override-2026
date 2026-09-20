#include "main.h"
#include <cmath>

// ============================================================================
// PID / SLEW CONSTANTS
// TODO(tune): carried over from last season's different robot — every
// number here needs retuning for this year's weight/gearing.
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
// TODO(missing): all three are empty stubs — write actual routines once
// driving and placement mechanics are dialed in.
// ============================================================================
void auton_skills() {}
void auton_button_1() {}
void auton_button_2() {}

// ============================================================================
// PID TUNER TEST MOVE
// Drives forward 24in (exercises Drive PID), then turns 90deg (exercises
// Turn PID — the IMU/rotation-based one). Press B in opcontrol while the
// tuner is on (X toggles it) to run this; edit the numbers below to test
// whatever distance/angle you're actually tuning against.
// ============================================================================
void tune_test() {
  chassis.pid_drive_set(24_in, 90, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 90, true);
  chassis.pid_wait();
}

// ============================================================================
// DRIVETRAIN CALIBRATION
// Two test moves for measuring the real numbers behind main.cpp's chassis
// constructor (wheel diameter, external gear ratio) and globals.hpp's
// ODOM_HORIZONTAL_WHEEL_DIAMETER/OFFSET — all four are still unmeasured
// guesses (TODO(verify)). Bound to controller buttons in main.cpp; results
// print to the brain (line 5) and controller, and every tick of both moves
// also lands in /usd/log.csv (sdlog.cpp).
//
// calibrate_straight() needs exactly ONE number back from a human: the real
// distance driven, read with a tape measure. Nothing on this robot knows
// true linear distance on its own — every distance number it can report
// (drive encoders, tracking wheel) is only as good as the wheel diameter
// it's assuming, which is the thing being measured here. No way around
// that without an external reference this robot doesn't have (e.g. a
// distance sensor aimed at a fixed wall).
//
// calibrate_spin(), by contrast, needs NO human measurement. A pure
// in-place spin has zero real sideways travel, so whatever lateral inches
// the tracking wheel reports during one is entirely explained by
// ODOM_HORIZONTAL_OFFSET being wrong — fully self-checkable using the IMU's
// own rotation count (an independent, trusted sensor) as ground truth for
// how far it actually turned.
//
// TODO(verify): also just count the actual teeth on the two meshed drive
// gears — ten seconds, and it settles the gear-ratio half of the drive
// unknown for free, leaving only wheel diameter to solve for from
// calibrate_straight()'s measurement.
// ============================================================================

// Raw encoder degrees, not inches — inches would bake in the very wheel
// diameter/gear ratio this is trying to measure. 3600 = 10 motor shaft
// rotations, long enough to tape-measure precisely without needing a huge
// stretch of open floor.
constexpr int CALIBRATE_DRIVE_DEGREES = 3600;
constexpr int CALIBRATE_DRIVE_SPEED = 60;  // slow and controlled — this is open-loop, not a PID move

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
  master.print(0, 0, "drv%.1f trk%.1f in", drive_in, tracker_in);
  pros::screen::print(TEXT_MEDIUM, 5, "CALIB straight: drive=%.2fin tracker=%.2fin -- tape-measure real distance",
                       drive_in, tracker_in);
  // Report the real measured distance (call it M) back, and both constants
  // correct by simple proportion — no need to re-derive the formulas:
  //   new wheel diameter   = old wheel diameter   * (M / drive_in)
  //   new tracker diameter = old tracker diameter * (M / tracker_in)
}

// More rotations = more averaging = less noise in the offset estimate, at
// the cost of a longer test. 8 is a reasonable start.
constexpr double CALIBRATE_SPIN_ROTATIONS = 8.0;
constexpr int CALIBRATE_SPIN_SPEED = 70;

void calibrate_spin() {
  chassis.drive_imu_reset();
  horizontal_tracker.reset();

  // ez::raw: commands the literal target angle, not "shortest path" (which
  // would see 8*360 as a no-op back to 0 and never actually spin).
  chassis.pid_turn_set(CALIBRATE_SPIN_ROTATIONS * 360.0, CALIBRATE_SPIN_SPEED, ez::raw);
  chassis.pid_wait();

  double actual_rotation_deg = chassis.imu.get_rotation();  // ground truth — IMU, not encoders
  double lateral_in = horizontal_tracker.get();              // should be ~0 for a perfect pure spin
  double radians = actual_rotation_deg * (M_PI / 180.0);
  double offset_estimate = radians != 0 ? lateral_in / radians : 0;

  master.print(0, 0, "off~%.2fin rot%.0f", offset_estimate, actual_rotation_deg);
  pros::screen::print(TEXT_MEDIUM, 5, "CALIB spin: rot=%.1fdeg lateral=%.2fin -> offset~%.3fin",
                       actual_rotation_deg, lateral_in, offset_estimate);
  // offset_estimate scales linearly with whatever the tracker's true
  // diameter turns out to be from calibrate_straight() — if that constant
  // gets corrected later, rescale this by (new tracker diameter / old
  // tracker diameter) instead of re-running the spin.
}
