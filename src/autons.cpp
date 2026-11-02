#include "main.h"

// TODO(tune): last season's numbers for a different robot, not measured on this one yet.
void default_constants() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // turning during odom moves
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // turning during boomerang moves

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);

  chassis.slew_turn_constants_set(5_deg, 50);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.pid_turn_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.odom_turn_bias_set(0.9);             // 1.0 = turns fully prioritized over driving
  chassis.odom_look_ahead_set(7_in);           // how far ahead on the path it aims
  chassis.odom_boomerang_distance_set(16_in);  // max distance of the boomerang carrot point
  chassis.odom_boomerang_dlead_set(0.625);     // how aggressive boomerang ends are
}

// Keeps calling lift::update(0) until a go_to_height() move settles -- the
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
// Absolute positioning: (x, y) in inches from the start, +y = the way the robot faces at
// the start, +x = to its right. Odom moves turn toward each point on their own.
void auton_button_1() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);  // starting pose
  chassis.pid_odom_set({{0_in, 5_in}, ez::fwd, 110}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, -5_in}, ez::rev, 110}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, 0_in}, ez::fwd, 110}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, -5_in}, ez::rev, 110}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{8.5_in, 11_in, 55_deg}, ez::fwd, 90}, true);  // the goal
  lift::go_to_height(5.5);  // alliance goal
  lift_wait();
  chassis.pid_wait();
  lift::go_to_height(3.0);
  lift_wait();

  claw::open();   // drop the pin into the goal
  pros::delay(200);
  chassis.pid_odom_set({{-3_in, 9_in}, ez::rev, 100}, true);
  lift::go_to_height(3.0);  // cup pickup
  lift_wait();
  chassis.pid_wait();
  chassis.pid_turn_set({7.5_in,-3_in},fwd,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{7.5_in, -3_in,170_deg}, ez::fwd, 90}, true);  // the cup
  chassis.pid_wait();
  claw::close();  // grab the cup
  pros::delay(100);
  chassis.pid_odom_set({{-5_in, 15_in}, ez::rev, 100}, true);
  lift::go_to_height(12.0);  // lift the cup up a bit so it doesn't scrape the ground while driving and lifts to goal +pin height
  lift_wait();
  chassis.pid_wait();
  chassis.pid_turn_set({8.5,11.5},fwd,90);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{8.5_in, 11.5_in}, ez::fwd, 80}, true);  // the goal
  chassis.pid_wait();
  lift::go_to_height(10.0);
  lift_wait();
  claw::open();  // drop the cup into the goal
  pros::delay(200);
  chassis.pid_odom_set({{-5_in, 15_in}, ez::rev, 100}, true);
  chassis.pid_wait();
  chassis.pid_turn_set({-3,45},fwd,90);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-3_in, 45_in}, ez::fwd, 100}, true);
  chassis.pid_wait();
}

// "Cup+Goal L" is the reflection of auton_button_1() across the robot's
// starting forward axis: x coordinates and explicit headings are negated.
void auton_button_1_other_side() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);  // starting pose
  chassis.pid_odom_set({{0_in, 5_in}, ez::fwd, 110}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, -5_in}, ez::rev, 110}, true);
  chassis.pid_wait();
  chassis.pid_odom_set({{-11_in,11_in, -80_deg}, ez::fwd, 90}, true);  // the goal
  lift::go_to_height(5.5);  // alliance goal
  lift_wait();
  chassis.pid_wait();
  lift::go_to_height(3.0);
  lift_wait();

  claw::open();  // drop the pin into the goal
  pros::delay(200);
  chassis.pid_odom_set({{8_in, 5_in}, ez::rev, 100}, true);
  lift::go_to_height(3.0);  // cup pickup
  lift_wait();
  chassis.pid_wait();
  chassis.pid_odom_set({{{-4_in, 3_in}, ez::fwd, 90},
                      {{-8_in, -7_in, -165_deg}, ez::fwd, 90}}, true);
  chassis.pid_wait();
  claw::close();  // grab the cup
  pros::delay(100);
  chassis.pid_odom_set({{5_in, 10_in}, ez::rev, 100}, true);
  lift::go_to_height(12.0);  // lift the cup so it does not scrape while driving
  lift_wait();
  chassis.pid_wait();
  chassis.pid_turn_set({-11_in, 7_in}, fwd, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-11_in, 7_in}, ez::fwd, 70}, true);  // the goal
  chassis.pid_wait();
  lift::go_to_height(10.0);
  lift_wait();
  claw::open();  // drop the cup into the goal
  pros::delay(200);
  chassis.pid_odom_set({{5_in, 15_in}, ez::rev, 100}, true);
  chassis.pid_wait();
  chassis.pid_turn_set({3, 45}, fwd, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{3_in, 45_in}, ez::fwd, 100}, true);
  chassis.pid_wait();
}

// "Drive Test" in the selector: drives 24in, then turns 90deg -- for checking PID constants.
void tune_test() {
  chassis.pid_drive_set(24_in, 90, true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 90, true);
  chassis.pid_wait();
}
