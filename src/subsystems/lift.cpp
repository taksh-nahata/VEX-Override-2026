// Only needs the plain PROS API, not the rest of EZ-Template
// (chassis/drive.hpp is by far the biggest header in this project) or
// main.h's other includes, so we only pull in what this file actually
// uses to keep compile time down.
#include "api.h"
#include "globals.hpp"
#include "subsystems/lift.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lift {

// ============================================================================
// HARDWARE
// 2 motors on the second four-bar (1:6 external reduction), sharing one
// physical shaft -- not separate gearing per side like the old crooked
// 2-motor lift, so these two can't get out of sync with each other.
// move()/brake_both()/current_ma() below just command/read both. Real
// moves (go_to_height() and everything built on it) are driven entirely
// by the claw distance sensor, converted straight to inches -- no
// rotation-sensor/degree math, no calibration constant. The rotation
// sensor still exists for position() (main.cpp's anti-tip -- something
// the distance sensor can't tell you, since it measures clearance to
// whatever's below, not the arm's own angle) and for idle hold below
// (it's not noisy the way the distance sensor is, and holding steady
// doesn't need to know what's on the ground anyway).
//
// Plain pros::Motor x2, not pros::MotorGroup -- MotorGroup hit an
// undefined-reference link error against this project's compiled PROS
// library (pros::rtos::Mutex), the same class of header/library
// mismatch that bit LVGL earlier this project. Two motors moved
// together needs nothing fancier than calling the same thing on both.
// ============================================================================
pros::Motor motor(PORT_LIFT, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor motor_2(PORT_LIFT_2, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Rotation rotation(PORT_LIFT_ROTATION);
pros::Distance claw_distance(PORT_CLAW_DISTANCE);

void move(int voltage) {
  motor.move(voltage);
  motor_2.move(voltage);
}

constexpr double MM_PER_IN = 25.4;

// ============================================================================
// TUNABLE CONSTANTS — grep "TODO(tune)" for everything that still needs a
// real number off the actual robot. Height presets are plain numbers
// inline in each go_to_*() function below, not up here -- there's only
// one of each, so there's nothing gained by naming them.
// ============================================================================

// TODO(tune): motor speed per inch of error, and the cap on that speed.
constexpr double SEEK_GAIN = 15.0;
constexpr int SEEK_SPEED = 100;

// TODO(tune): a constant push against gravity, added whenever we're
// commanding the lift upward, so the speed above only has to correct
// leftover error instead of fighting the same predictable sag every tick.
constexpr int GRAVITY_HOLD = 15;

constexpr int STICK_DEADBAND = 10;

// TODO(tune): how close (inches) counts as "arrived" for a go_to_*()
// move, and also how close counts as "touching down" while lowering
// manually.
constexpr double ARRIVED_TOLERANCE_IN = 0.4;

// TODO(tune): how far (degrees, rotation sensor) the lift can drift from
// its idle-hold target before we correct it, and how hard. Added after
// testing showed correcting every single tick made the lift noticeably
// easy to push by hand -- the motor's own brake mode resists a hand-push
// a lot harder than a correction every ~20ms does, so below this we just
// let brake mode hold it and only step in for the slow gravity sag brake
// mode can't stop alone. This used to compare distance-sensor readings
// instead of rotation degrees -- that sensor's own jitter was enough to
// look like drift and made the lift randomly correct itself the instant
// R1/R2 was released, even sitting still.
constexpr double HOLD_DEADBAND_DEG = 3.0;
constexpr double HOLD_GAIN = 2.0;

// TODO(tune)/TODO(verify): current (mA) that means the lift just hit
// something solid. The ceiling threshold is set higher on purpose --
// raising fights gravity and lowering doesn't, so normal raising
// current runs higher than normal lowering current even with nothing
// in the way. These were tuned for a single motor -- with the load now
// split across two motors on the same shaft, one motor's current for
// the same stall could look different, so re-check both against the
// debug screen's live mA reading.
constexpr std::int32_t CONTACT_CURRENT_MA = 1500;
constexpr std::int32_t CEILING_CURRENT_MA = 2200;

// How many ticks in a row current has to stay high before we actually
// call it a contact stop. Every motor gives a brief current spike just
// from starting to move under load, and without this debounce that
// spike alone would trip a false stop the instant R1/R2 is pressed.
constexpr int CONTACT_DEBOUNCE_TICKS = 5;

// ============================================================================
// STATE
// ============================================================================
bool homing = false;
double target_in = 0;            // height we're homing toward
bool holding = false;             // idle-hold target has been captured for this hold
double hold_target_deg = 0;       // rotation-sensor degrees, captured the instant idle hold engages
bool placing_contact = false;    // stopped while lowering (distance close, or current spiked)
bool at_ceiling = false;         // current-based stop fired while raising
int contact_high_ticks = 0;
int ceiling_high_ticks = 0;

// ============================================================================
// QUERIES
// ============================================================================

void initialize() {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  motor_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotation.reset_position();
}

double position() {
  return rotation.get_position() / 100.0;  // sensor reports centidegrees
}

// Just the first motor's current (MotorGroup::get_current_draw()
// defaults to index 0) -- fine since both motors are rigidly on the
// same shaft and share the load, but a stall now splits its current
// across two motors instead of one, so CONTACT_CURRENT_MA/
// CEILING_CURRENT_MA below may read differently than they used to.
std::int32_t current_ma() {
  return motor.get_current_draw();
}

double claw_distance_in() {
  return claw_distance.get() / MM_PER_IN;
}

bool touched_down() {
  return placing_contact;
}

bool at_ceiling_now() {
  return at_ceiling;
}

bool is_homing() {
  return homing;
}

// Reads the distance sensor, rejecting the "nothing there" case and any
// error reading, and converts straight to inches. Every homing move
// goes through this -- one place that decides what counts as a real
// reading.
bool sensed(double& in_out) {
  std::int32_t mm = claw_distance.get();
  if (mm <= 0 || mm >= 9999) return false;
  in_out = mm / MM_PER_IN;
  return true;
}

// ============================================================================
// PUBLIC CONTROL
// go_to_height() is the one real move: pass a target in inches and it
// rises/lowers until the distance sensor reads that. Everything else
// below is just a named shortcut for a number someone would otherwise
// have to remember. If the sensor can't see anything, we just don't
// move -- no guessed fallback height.
// ============================================================================

void go_to_height(double target) {
  homing = true;
  holding = false;
  target_in = target;
}

void go_to_floor() {
  go_to_height(0);
}

// Alliance Goal is 3.25in tall (game manual Appendix B) -- guess includes
// a little clearance on top so the pin drops in without scraping.
void go_to_alliance_goal() {
  go_to_height(4.0);
}

// Neutral (quadrant) Goal is 5.8in tall -- same clearance idea.
void go_to_neutral_goal() {
  go_to_height(6.5);
}

// Center Goal is 8.7in tall -- same clearance idea.
void go_to_center_goal() {
  go_to_height(9.5);
}

// 1st/2nd/3rd pin stacked on whatever Goal you're already at -- all
// guesses, pace them out against a real stack.
void go_to_pin_1() {
  go_to_height(9.5);
}

void go_to_pin_2() {
  go_to_height(13.6);
}

void go_to_pin_3() {
  go_to_height(20.1);
}

// A Pin nested inside a Cup measures ~10in as one unit (bench-measured
// 2026-09-27) -- a little clearance on top of that.
void go_to_pin_on_cup() {
  go_to_height(5.3);
}

// Just enough to clear the ground and line the preload pin up with a
// cup's opening while driving to it -- pure guess.
void go_to_cup_drop() {
  go_to_height(8.5);
}

void update(int stick) {
  double in;
  bool have_reading = sensed(in);

  if (std::abs(stick) > STICK_DEADBAND) {
    homing = false;
    holding = false;  // let go of the old hold target -- we'll capture a new one next time it idles

    if (stick < 0) {
      at_ceiling = false;
      ceiling_high_ticks = 0;

      bool distance_close = have_reading && in <= ARRIVED_TOLERANCE_IN;
      bool current_high = current_ma() > CONTACT_CURRENT_MA;
      contact_high_ticks = current_high ? contact_high_ticks + 1 : 0;
      placing_contact = distance_close || contact_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (placing_contact) {
        // True stop -- doesn't open the claw. Dropping a pin can't be
        // undone, so that's still a deliberate, separate button press.
        move(0);
        return;
      }
    } else {
      placing_contact = false;
      contact_high_ticks = 0;

      bool current_high = current_ma() > CEILING_CURRENT_MA;
      ceiling_high_ticks = current_high ? ceiling_high_ticks + 1 : 0;
      at_ceiling = ceiling_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (at_ceiling) {
        move(0);
        return;
      }
    }

    move(stick);
    return;
  }

  // Reset here too, not just above -- these used to only get cleared
  // inside the active-stick branch, so letting go of R1/R2 could leave
  // TOUCHED or CEILING showing on the screen long after it was true.
  placing_contact = false;
  at_ceiling = false;
  contact_high_ticks = 0;
  ceiling_high_ticks = 0;

  if (homing) {
    if (!have_reading) {
      // Nothing to measure against -- stop rather than guess.
      homing = false;
      move(0);
      return;
    }
    double error = target_in - in;
    if (std::fabs(error) <= ARRIVED_TOLERANCE_IN) {
      homing = false;
      move(0);
      return;
    }
    int speed = std::clamp(static_cast<int>(error * SEEK_GAIN), -SEEK_SPEED, SEEK_SPEED);
    if (speed > 0) speed += GRAVITY_HOLD;
    move(speed);
    return;
  }

  // Idle hold: capture wherever the lift is (rotation-sensor degrees,
  // not the distance sensor -- see HOLD_DEADBAND_DEG) the instant both
  // buttons are let go, then mostly leave it to the motor's own brake
  // mode -- only step in once it's drifted past the deadband.
  if (!holding) {
    holding = true;
    hold_target_deg = position();
  }
  double drift_deg = hold_target_deg - position();
  if (std::fabs(drift_deg) > HOLD_DEADBAND_DEG) {
    int speed = std::clamp(static_cast<int>(drift_deg * HOLD_GAIN), -SEEK_SPEED, SEEK_SPEED);
    if (speed > 0) speed += GRAVITY_HOLD;
    move(speed);
  } else {
    move(0);
  }
}

}  // namespace lift
