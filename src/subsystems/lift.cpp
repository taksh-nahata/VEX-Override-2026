// Only needs ez::PID, not the rest of EZ-Template (chassis/drive.hpp is by
// far the biggest header in this project) or main.h's other includes, so
// pull in just what's used to keep this file's compile time down.
#include "api.h"
#include "EZ-Template/PID.hpp"
#include "globals.hpp"
#include "subsystems/lift.hpp"
#include <algorithm>
#include <cmath>

namespace lift {

pros::Motor left_motor(PORT_LIFT_L, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor right_motor(PORT_LIFT_R, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// TODO: retune once the DR4B is built and weighed.
ez::PID height_pid(0.4, 0.0, 1.0, 0);
ez::PID sync_pid(0.2, 0.0, 0.0, 0);

constexpr int STICK_DEADBAND = 10;
constexpr double FLOOR_TOLERANCE_DEG = 10.0;

bool homing = false;

void initialize() {
  left_motor.tare_position();
  right_motor.tare_position();
  left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

double position() {
  return (left_motor.get_position() + right_motor.get_position()) / 2.0;
}

// Whichever side is physically lower right now — used for the floor clamp
// so one side can't keep sinking below its own start just because the
// average of both sides hasn't hit 0 yet.
double lowest_position() {
  return std::min(left_motor.get_position(), right_motor.get_position());
}

// Keeps both sides level regardless of who's driving the lift (manual or PID).
double sync_correction() {
  double skew = left_motor.get_position() - right_motor.get_position();
  return sync_pid.compute_error(-skew, skew);
}

void go_to_floor() {
  homing = true;
  height_pid.target_set(0);
}

void update(int stick) {
  double correction = sync_correction();

  if (std::abs(stick) > STICK_DEADBAND) {
    homing = false;

    // Never drive below where the lift was at boot (position 0, set by
    // tare_position() in initialize()) — ignore further "down" commands
    // once EITHER side gets there, instead of grinding the mechanism
    // against itself while waiting for the average to catch up.
    if (stick < 0 && lowest_position() <= 0) stick = 0;

    left_motor.move(stick - correction);
    right_motor.move(stick + correction);
    return;
  }

  if (homing) {
    double out = height_pid.compute(position());
    left_motor.move(out - correction);
    right_motor.move(out + correction);
    if (std::fabs(position()) < FLOOR_TOLERANCE_DEG) homing = false;
    return;
  }

  left_motor.move(0);
  right_motor.move(0);
}

}  // namespace lift
