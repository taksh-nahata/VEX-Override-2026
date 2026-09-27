#include "main.h"
#include <cmath>

// TODO(tune): last season's numbers for a different robot, not measured on this one yet.
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

void auton_skills() {}  // TODO(missing): skills has its own timing, not planned yet

// Keeps calling lift::update(0) until a go_to_pin_1/2/3()/go_to_cup_drop() move settles -- the
// lift only actually moves while something calls update(), unlike the chassis's own PID.
void lift_wait(std::uint32_t timeout_ms = 1000) {
  std::uint32_t start = pros::millis();
  while (lift::is_homing() && pros::millis() - start < timeout_ms) {
    lift::update(0);
    pros::delay(10);
  }
}

// AUTON: BUTTON 1 -- "Cup+Goal", the current main plan. All distances/angles below are
// placeholders -- pace them out for real. TODO(verify): drop-then-grab assumed to be a plain
// open-then-close on the same claw, not confirmed against the real mechanism.
void auton_button_1() {
  claw::close();          // preload is already in the claw at match start
  lift::go_to_cup_drop();
  lift_wait();
  chassis.pid_drive_set(7, 70, true);  // drive to the cup
  chassis.pid_wait();

  claw::open();   // drop the pin into the cup
  pros::delay(300);
  claw::close();  // grab the whole cup
  pros::delay(300);

  chassis.pid_drive_set(-6, 60, true);  // back off so the turn doesn't drag the cup
  chassis.pid_wait();
  chassis.pid_turn_set(180, 90, true);  // spin around
  chassis.pid_wait();

  chassis.pid_drive_set(9, 70, true);  // drive to the goal
  chassis.pid_wait();
  lift::go_to_cup_on_goal();  // it's the whole cup+pin unit going down, not a bare pin
  lift_wait();
  claw::open();  // place it
}

// AUTON: BUTTON 2 -- "Loader x2", the earlier plan, kept as a fallback. Scores the preload plus
// 2 Loader cycles for 3 pins -- going for the 12-point auto bonus, not the 7-pin Autonomous Win
// Point (not realistic without an intake in 15 seconds). All distances/angles are placeholders.
// TODO(verify): does the Loader need the lift at a specific height, or is floor height fine?
void auton_button_2() {
  lift::go_to_pin_1();  // empty goal height
  lift_wait();
  chassis.pid_drive_set(12, 90, true);  // drive to the goal
  chassis.pid_wait();
  claw::open();  // drop the preload
  pros::delay(200);

  chassis.pid_turn_set(90, 90, true);  // turn to the Loader
  chassis.pid_wait();
  chassis.pid_drive_set(12, 90, true);  // drive to the Loader
  chassis.pid_wait();
  claw::close();  // grab pin #2
  pros::delay(200);
  chassis.pid_drive_set(-12, 90, true);  // back out of the Loader
  chassis.pid_wait();
  chassis.pid_turn_set(-90, 90, true);  // turn back to the goal
  chassis.pid_wait();
  lift::go_to_pin_2();  // goal now has 1 pin on it
  lift_wait();
  chassis.pid_drive_set(12, 90, true);  // drive to the goal
  chassis.pid_wait();
  claw::open();  // stack pin #2
  pros::delay(200);

  chassis.pid_turn_set(90, 90, true);  // turn to the Loader
  chassis.pid_wait();
  chassis.pid_drive_set(12, 90, true);  // drive to the Loader
  chassis.pid_wait();
  claw::close();  // grab pin #3
  pros::delay(200);
  chassis.pid_drive_set(-12, 90, true);  // back out of the Loader
  chassis.pid_wait();
  chassis.pid_turn_set(-90, 90, true);  // turn back to the goal
  chassis.pid_wait();
  lift::go_to_pin_3();  // goal now has 2 pins on it
  lift_wait();
  chassis.pid_drive_set(12, 90, true);  // drive to the goal
  chassis.pid_wait();
  claw::open();  // stack pin #3
}

// Runs while the drivetrain PID tuner (main.cpp's X/B) is on, so there's an actual move to judge
// the live values against: drives 24in, then turns 90deg.
void tune_test() {
  chassis.pid_drive_set(24_in, 90, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 90, true);
  chassis.pid_wait();
}

// DRIVETRAIN CALIBRATION -- for the numbers we still don't know: the gear ratio in main.cpp's
// chassis constructor, and ODOM_HORIZONTAL_OFFSET in globals.hpp. Not bound to a button right
// now (freed up for the lift presets). Results print to brain line 5 + controller line 2, and
// every tick also lands in /usd/log.csv.

constexpr int CALIBRATE_DRIVE_DEGREES = 3600;  // 10 motor shaft rotations, raw not inches (inches
                                                // would assume the gear ratio being measured)
constexpr int CALIBRATE_DRIVE_SPEED = 60;      // open-loop, not a PID move

// Tape-measure the real distance driven (M) and report it:
//   new gear ratio = old gear ratio * (M / drive_in)
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

// A pure in-place spin has zero real sideways travel, so lateral drift the tracker picks up
// during one is entirely ODOM_HORIZONTAL_OFFSET's fault -- no tape measure needed.
constexpr double CALIBRATE_SPIN_ROTATIONS = 8.0;
constexpr int CALIBRATE_SPIN_SPEED = 35;  // slower than a real turn -- see git history 2026-09-26

void calibrate_spin() {
  chassis.drive_imu_reset();
  chassis.odom_xyt_set(0, 0, 0);
  horizontal_tracker.reset();

  chassis.pid_turn_set(CALIBRATE_SPIN_ROTATIONS * 360.0, CALIBRATE_SPIN_SPEED, ez::raw);  // ez::raw
                                                                                           // = literal target, not shortest path
  chassis.pid_wait();

  double actual_rotation_deg = chassis.imu.get_rotation();
  double lateral_in = horizontal_tracker.get();
  double radians = actual_rotation_deg * (M_PI / 180.0);
  double offset_estimate = radians != 0 ? lateral_in / radians : 0;

  // odom_x/odom_y (not lateral_in again) are what actually reflect a fix to the offset constant
  master.print(0, 2, "odX%.2f odY%.2f", chassis.odom_x_get(), chassis.odom_y_get());
  pros::screen::print(TEXT_MEDIUM, 5,
                       "CALIB spin: rot=%.1fdeg raw_lateral=%.2fin off_est~%.3fin  odom_x=%.2f odom_y=%.2f",
                       actual_rotation_deg, lateral_in, offset_estimate, chassis.odom_x_get(), chassis.odom_y_get());
}
