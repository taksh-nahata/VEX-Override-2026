// Only needs ez::PID, not the rest of EZ-Template (chassis/drive.hpp is by
// far the biggest header in this project) or main.h's other includes, so
// pull in just what's used to keep this file's compile time down.
#include "api.h"
#include "EZ-Template/PID.hpp"
#include "globals.hpp"
#include "subsystems/lift.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lift {

pros::Motor left_motor(PORT_LIFT_L, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor right_motor(PORT_LIFT_R, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// TODO: retune once the DR4B is built and weighed.
ez::PID height_pid(0.4, 0.0, 1.0, 0);
ez::PID sync_pid(0.2, 0.0, 0.0, 0);

constexpr int STICK_DEADBAND = 10;
constexpr double FLOOR_TOLERANCE_DEG = 10.0;

// EXPERIMENTAL — how hard the motors are working while lowering is used to
// detect "the pin just landed on something solid" (the stack/goal), same
// idea as the abandoned stack-height idea but now in a scenario where it's
// actually sound: something IS guaranteed to eventually be under a pin
// that's being lowered onto a goal, unlike raising into open air. This is
// still a totally unverified guess — print left_current_ma()/
// right_current_ma() to the screen and watch them while manually lowering
// onto a real stack to find the real number (normal descending load vs. the
// spike when it actually lands), rather than trusting this value.
constexpr std::int32_t CONTACT_CURRENT_MA = 1500;

bool homing = false;
bool placing_contact = false;
bool floor_limit_enabled = true;

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

std::int32_t left_current_ma() {
  return left_motor.get_current_draw();
}

std::int32_t right_current_ma() {
  return right_motor.get_current_draw();
}

// True for the tick(s) after update() last stopped a downward move because
// of a current spike (i.e. it thinks a pin just landed on something). NOT
// the same thing as the floor limit below — this is current-based, the
// floor limit is position-based, they're independent checks.
bool touched_down() {
  return placing_contact;
}

// Toggle for the boot-position floor limit, so it can be switched off while
// troubleshooting (e.g. the crooked-lift issue) without editing code, and
// back on afterward. See main.cpp for which button toggles this.
void toggle_floor_limit() {
  floor_limit_enabled = !floor_limit_enabled;
}

bool floor_limit_on() {
  return floor_limit_enabled;
}

// Keeps both sides level regardless of who's driving the lift (manual or
// PID). Capped so it can only ever nudge, never override — an uncapped
// correction grows with however out-of-sync the sides currently are, and
// once it's bigger than the commanded stick value it flips that side's
// sign entirely: e.g. holding R1 (both sides should rise) but a large
// correction pushes stick - correction negative, so the left side drops
// instead. That's the "R1/R2 sometimes goes the wrong way" bug — it gets
// worse the more the two sides have drifted apart since boot, which is why
// it's intermittent rather than every time.
double sync_correction() {
  double skew = left_motor.get_position() - right_motor.get_position();
  double correction = sync_pid.compute_error(-skew, skew);
  constexpr double MAX_CORRECTION = 30;  // TODO: tune — smallest value that still keeps both sides level
  return std::clamp(correction, -MAX_CORRECTION, MAX_CORRECTION);
}

void go_to_floor() {
  homing = true;
  height_pid.target_set(0);
}

void update(int stick) {
  double correction = sync_correction();

  if (std::abs(stick) > STICK_DEADBAND) {
    homing = false;

    if (stick < 0) {
      // Never drive below where the lift was at boot (position 0, set by
      // tare_position() in initialize()) — ignore further "down" commands
      // once EITHER side gets there, instead of grinding the mechanism
      // against itself while waiting for the average to catch up. A true
      // stop, not just zeroing the driver's input — correction doesn't get
      // to sneak the motors past this either. Toggleable (see main.cpp).
      if (floor_limit_enabled && lowest_position() <= 0) {
        placing_contact = false;
        left_motor.move(0);
        right_motor.move(0);
        return;
      }

      // Placing: stop lowering the instant something solid is under the
      // pin, instead of continuing to grind into it. Also a true stop.
      // Does NOT open the claw — that's still a deliberate, separate
      // button press (see the header comment on why release stays manual).
      placing_contact = left_current_ma() > CONTACT_CURRENT_MA || right_current_ma() > CONTACT_CURRENT_MA;
      if (placing_contact) {
        left_motor.move(0);
        right_motor.move(0);
        return;
      }
    } else {
      placing_contact = false;
    }

    left_motor.move(stick - correction);
    right_motor.move(stick + correction);
    return;
  }

  // Neither button held — this used to leave placing_contact stuck at
  // whatever it last was (it was only ever touched above, which only runs
  // while actively lowering/raising), so the screen could keep showing
  // TOUCHED long after you'd let go.
  placing_contact = false;

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
