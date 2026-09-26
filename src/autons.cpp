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
// auton_button_1() is our first real match auto (see below). Skills is its
// own separate game mode with its own timing, not just a longer version of
// this, so it's staying a stub until we're actually ready to plan that.
// auton_button_2() is open for a second variant later -- e.g. the other
// starting side, or a safer fallback if button_1 turns out too tight on
// time once it's tested for real.
// ============================================================================
void auton_skills() {}
void auton_button_2() {}

// Blocks until the current lift preset move actually settles. The lift
// only moves while something calls lift::update() -- it's not on its own
// background task the way the chassis's drive PID is -- so auton has to
// keep feeding it ticks itself while it waits, the same job opcontrol()'s
// loop normally does. The timeout is a safety net so one preset that never
// quite settles can't eat the whole 15-second auto by itself.
void lift_wait(std::uint32_t timeout_ms = 1000) {
  std::uint32_t start = pros::millis();
  while (lift::is_homing() && pros::millis() - start < timeout_ms) {
    lift::update(0);
    pros::delay(10);
  }
}

// ============================================================================
// AUTON: BUTTON 1 -- first real auto, scores the preload plus 2 Loader
// cycles for 3 pins total. We're going for the Loader instead of picking
// pins up off the open field on purpose: with no intake, every grab needs
// precise alignment, and the Loader sits in the same fixed spot every
// match, so it's something we can actually aim at reliably without vision.
// Chasing the full 7-pin Autonomous Win Point isn't realistic without an
// intake in 15 seconds -- this is aimed at the much easier 12-point auto
// bonus (just outscoring the other alliance's auto) instead.
//
// TODO(measure): every DRIVE_*/TURN_* constant below is a placeholder.
// Pace out (or measure) the real distances/angles from the actual starting
// tile to the goal and to the Loader and fill these in -- inventing
// plausible-sounding numbers without measuring the real field would just
// be wrong. TODO(verify): does grabbing from the Loader need the lift at a
// specific height, or is floor height fine? If it needs its own height,
// that's a 4th preset the same way go_to_pin_1/2/3() work.
//
// TODO(tune): 15 seconds is tight for 3 full Loader cycles once realistic
// PID move times are accounted for, especially with Drive/Turn PID still
// untuned -- time this for real once the distances below are filled in,
// and don't be surprised if it needs cutting back to 2 pins (preload +
// 1 cycle) to actually fit.
constexpr double DRIVE_TO_GOAL_IN = 12;
constexpr double TURN_TO_LOADER_DEG = 90;
constexpr double DRIVE_TO_LOADER_IN = 12;
constexpr double TURN_TO_GOAL_DEG = -90;
constexpr int AUTON_DRIVE_SPEED = 90;
constexpr int AUTON_TURN_SPEED = 90;

void auton_button_1() {
  // Preload starts secured in the claw already -- first move is straight
  // to scoring it, no grab needed.
  lift::go_to_pin_1();  // empty goal height
  lift_wait();
  chassis.pid_drive_set(DRIVE_TO_GOAL_IN, AUTON_DRIVE_SPEED, true);
  chassis.pid_wait();
  claw::open();
  pros::delay(200);  // let the pin actually clear the claw before we move again

  // Loader cycle #1 -> pin #2, stacked on top of the preload.
  chassis.pid_turn_set(TURN_TO_LOADER_DEG, AUTON_TURN_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(DRIVE_TO_LOADER_IN, AUTON_DRIVE_SPEED, true);
  chassis.pid_wait();
  claw::close();  // grab from the Loader
  pros::delay(200);
  chassis.pid_drive_set(-DRIVE_TO_LOADER_IN, AUTON_DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(TURN_TO_GOAL_DEG, AUTON_TURN_SPEED, true);
  chassis.pid_wait();
  lift::go_to_pin_2();  // goal now has 1 pin on it
  lift_wait();
  chassis.pid_drive_set(DRIVE_TO_GOAL_IN, AUTON_DRIVE_SPEED, true);
  chassis.pid_wait();
  claw::open();
  pros::delay(200);

  // Loader cycle #2 -> pin #3.
  chassis.pid_turn_set(TURN_TO_LOADER_DEG, AUTON_TURN_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(DRIVE_TO_LOADER_IN, AUTON_DRIVE_SPEED, true);
  chassis.pid_wait();
  claw::close();
  pros::delay(200);
  chassis.pid_drive_set(-DRIVE_TO_LOADER_IN, AUTON_DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(TURN_TO_GOAL_DEG, AUTON_TURN_SPEED, true);
  chassis.pid_wait();
  lift::go_to_pin_3();  // goal now has 2 pins on it
  lift_wait();
  chassis.pid_drive_set(DRIVE_TO_GOAL_IN, AUTON_DRIVE_SPEED, true);
  chassis.pid_wait();
  claw::open();
}

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

// ============================================================================
// PATH.JERRYIO IMPORT (WIP -- the team is still building this path)
//
// PATH.JERRYIO exports in centimeters, origin at the CENTER of the field.
// This chassis (and everywhere else in this project) uses inches, and we
// wanted the origin at the bottom-right corner instead -- the corner the
// team's starting near for this auto. Converting isn't just a shift:
//   new_x = 72 - (raw_x_cm / 2.54)   -- mirrored, not just shifted
//   new_y = (raw_y_cm / 2.54) + 72
// The mirror on X (not Y) is deliberate: in PATH.JERRYIO's exported
// coordinates, the bottom-right corner sits at the far +X, so a plain
// shift alone would leave "moving away from your own corner, into the
// field" reading as NEGATIVE X, which is backwards from how every other
// distance in this project already works (increasing = further from
// where you started). Mirroring X (and only X) fixes that without
// touching Y, since the bottom-right corner is already at the most
// negative Y, so a plain shift alone already makes "into the field"
// positive for Y.
//
// Headings needed their own transform, not just carried over -- mirroring
// only one axis flips left/right-facing directions but leaves up/down
// alone. Checked PATH.JERRYIO's own angle convention against the actual
// direction of travel between consecutive points in the raw file before
// trusting a formula (0 deg = facing the same way as +Y before the
// mirror, measured clockwise -- confirmed this matches by checking that
// the heading at the very first point lines up with which way the path
// actually heads from point 1 to point 2, and again partway through).
// The matching transform for that convention is: new_heading = (360 -
// raw_heading) % 360.
//
// TODO(verify): the position/heading transform above is checked against
// the raw file's own geometry, not against the real field yet -- run this
// once the path is finished and confirm the first few feet actually go
// where the team's PATH.JERRYIO picture shows before trusting the rest.
//
// TODO(verify): every point below is ez::fwd right now. The team asked
// separately how to make part of this path drive backwards instead of
// turning -- that's a per-point drive_direction (ez::fwd vs ez::rev, see
// the odom struct in EZ-Template/util.hpp), and needs the team to say
// which stretches of the finished path should be reverse, not something
// we can guess from the exported file alone.
//
// TODO(verify): speed values below (120, from the file) are carried over
// as-is -- haven't confirmed PATH.JERRYIO's speed units actually match
// what EZ-Template's max_xy_speed expects here.
void auton_jerryio_test() {
  chassis.pid_odom_pp_set(
      std::vector<odom>{
          {{9.156, 35.064, 90.0}, ez::fwd, 120},
          {{9.942, 35.021}, ez::fwd, 120},
          {{10.707, 34.843}, ez::fwd, 120},
          {{11.421, 34.513}, ez::fwd, 120},
          {{12.067, 34.065}, ez::fwd, 120},
          {{12.665, 33.552}, ez::fwd, 120},
          {{13.247, 33.022}, ez::fwd, 120},
          {{13.839, 32.502}, ez::fwd, 120},
          {{14.455, 32.013}, ez::fwd, 120},
          {{15.101, 31.563}, ez::fwd, 120},
          {{15.775, 31.156}, ez::fwd, 120},
          {{16.475, 30.794}, ez::fwd, 120},
          {{17.194, 30.475}, ez::fwd, 120},
          {{17.930, 30.195}, ez::fwd, 120},
          {{18.186, 30.315, 130.0}, ez::fwd, 120},
          {{17.740, 30.963}, ez::fwd, 120},
          {{17.294, 31.612}, ez::fwd, 120},
          {{16.848, 32.261}, ez::fwd, 120},
          {{16.402, 32.910}, ez::fwd, 120},
          {{15.933, 33.543, 330.0}, ez::fwd, 120},
          {{15.490, 34.193}, ez::fwd, 120},
          {{15.113, 34.884}, ez::fwd, 120},
          {{14.852, 35.625}, ez::fwd, 120},
          {{14.794, 36.407}, ez::fwd, 120},
          {{15.029, 37.153}, ez::fwd, 120},
          {{15.501, 37.780}, ez::fwd, 120},
          {{16.090, 38.302}, ez::fwd, 120},
          {{16.723, 38.769}, ez::fwd, 120},
          {{17.368, 39.222}, ez::fwd, 120},
          {{18.002, 39.688}, ez::fwd, 120},
          {{18.598, 40.186, 40.0}, ez::fwd, 120},
          {{18.598, 40.186, 40.0}, ez::fwd, 0},
      },
      true);
  chassis.pid_wait();
}
