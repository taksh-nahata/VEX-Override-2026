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
// One motor on the second four-bar (1:6 external reduction). Height
// control is driven entirely by the claw distance sensor (points down,
// sees whatever's under the claw) -- there's no rotation-sensor/degree
// math in the lift's own control anymore. The rotation sensor still
// exists for position(), which main.cpp's anti-tip uses to scale speed
// by how high the lift physically is (something the distance sensor
// can't tell you, since it measures clearance to whatever's below, not
// the arm's own angle).
// ============================================================================
pros::Motor motor(PORT_LIFT, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Rotation rotation(PORT_LIFT_ROTATION);
pros::Distance claw_distance(PORT_CLAW_DISTANCE);

// ============================================================================
// TUNABLE CONSTANTS — grep "TODO(tune)" for everything that still needs a
// real number off the actual robot.
// ============================================================================

// TODO(tune): motor speed per mm of error, and the cap on that speed.
constexpr double SEEK_GAIN = 0.6;
constexpr int SEEK_SPEED = 100;

// TODO(tune): a constant push against gravity, added whenever we're
// commanding the lift upward, so the speed above only has to correct
// leftover error instead of fighting the same predictable sag every tick.
constexpr int GRAVITY_HOLD = 15;

constexpr int STICK_DEADBAND = 10;

// TODO(tune): how close (mm) counts as "arrived" for a go_to_*() move,
// and also how close counts as "touching down" while lowering manually.
constexpr int ARRIVED_TOLERANCE_MM = 10;

// TODO(tune): how far (mm) the lift can drift from its idle-hold target
// before we correct it. Added after testing showed correcting every
// single tick made the lift noticeably easy to push by hand -- the
// motor's own brake mode resists a hand-push a lot harder than a
// correction every ~20ms does, so below this we just let brake mode
// hold it and only step in for the slow gravity sag brake mode can't
// stop alone.
constexpr int HOLD_DEADBAND_MM = 8;

// A Pin is 6.5in (165mm) tall, 1.6in (40mm) diameter -- official spec,
// game manual Appendix B ("Pin -"). Pins nest into each other when
// stacked (<SC2>: "partially or entirely nested"), so the real height
// one more stacked pin adds is less than its full 165mm -- the manual
// doesn't publish that overlap as a number. Using the full un-nested
// height here is a deliberate overshoot (safer to aim a bit high than
// into the stack) until it's checked against two real nested pins with
// a ruler.
constexpr int PIN_LAYER_MM = 165;
// TODO(tune): margin so the pin drops in without scraping -- not a
// spec'd number, just an engineering guess.
constexpr int PLACE_CLEARANCE_MM = 15;

// Measured on the bench 2026-09-27: a Pin sitting inside a Cup is about
// 10in (254mm) tall as one unit -- taller than a bare Pin since the Cup
// sits around it. go_to_cup_on_goal() places this whole nested unit, not
// a bare Pin, so it gets its own clearance number.
constexpr int CUP_WITH_PIN_MM = 254;

// TODO(tune): just enough to clear the ground and line the preload pin
// up with a cup's opening while driving to it. Pure guess.
constexpr int CUP_DROP_CLEARANCE_MM = 180;

// TODO(tune)/TODO(verify): current (mA) that means the lift just hit
// something solid. The ceiling threshold is set higher on purpose --
// raising fights gravity and lowering doesn't, so normal raising
// current runs higher than normal lowering current even with nothing
// in the way.
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
int target_mm = 0;              // clearance we're homing toward
bool holding = false;            // idle-hold target has been captured for this hold
int hold_target_mm = 0;          // clearance captured the instant idle hold engages
bool placing_contact = false;    // stopped while lowering (distance close, or current spiked)
bool at_ceiling = false;         // current-based stop fired while raising
int contact_high_ticks = 0;
int ceiling_high_ticks = 0;

// ============================================================================
// QUERIES
// ============================================================================

void initialize() {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotation.reset_position();
}

double position() {
  return rotation.get_position() / 100.0;  // sensor reports centidegrees
}

std::int32_t current_ma() {
  return motor.get_current_draw();
}

// mm from the claw down to whatever's directly under it. 9999 means the
// sensor can't see anything solid (out of range).
std::int32_t claw_distance_mm() {
  return claw_distance.get();
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

// Reads the distance sensor, rejecting the "nothing there" case (9999)
// and any error reading. Every homing/holding move goes through this --
// one place that decides what counts as a real reading.
bool sensed(int& mm_out) {
  std::int32_t mm = claw_distance_mm();
  if (mm <= 0 || mm >= 9999) return false;
  mm_out = mm;
  return true;
}

// Motor speed to close a clearance error of this many mm, gravity-assisted
// when moving up. Shared by homing and idle hold -- same math either way.
int seek_speed(int error_mm) {
  int speed = std::clamp(static_cast<int>(error_mm * SEEK_GAIN), -SEEK_SPEED, SEEK_SPEED);
  if (speed > 0) speed += GRAVITY_HOLD;
  return speed;
}

// ============================================================================
// PUBLIC CONTROL
// Every go_to_*() below is the same idea: read how far the claw is from
// whatever's under it right now, and rise/lower until it's
// `target_clearance_mm` above that instead. If the sensor can't see
// anything, we just don't move -- no guessed fallback height.
// ============================================================================

void go_to_clearance(int target_clearance_mm) {
  homing = true;
  holding = false;
  target_mm = target_clearance_mm;
}

void go_to_floor() {
  go_to_clearance(0);
}

void go_to_pin_1() {
  go_to_clearance(1 * PIN_LAYER_MM + PLACE_CLEARANCE_MM);
}

void go_to_pin_2() {
  go_to_clearance(2 * PIN_LAYER_MM + PLACE_CLEARANCE_MM);
}

void go_to_pin_3() {
  go_to_clearance(3 * PIN_LAYER_MM + PLACE_CLEARANCE_MM);
}

void go_to_cup_on_goal() {
  go_to_clearance(CUP_WITH_PIN_MM + PLACE_CLEARANCE_MM);
}

void go_to_cup_drop() {
  go_to_clearance(CUP_DROP_CLEARANCE_MM);
}

void update(int stick) {
  int mm;
  bool have_reading = sensed(mm);

  if (std::abs(stick) > STICK_DEADBAND) {
    homing = false;
    holding = false;  // let go of the old hold target -- we'll capture a new one next time it idles

    if (stick < 0) {
      at_ceiling = false;
      ceiling_high_ticks = 0;

      bool distance_close = have_reading && mm <= ARRIVED_TOLERANCE_MM;
      bool current_high = current_ma() > CONTACT_CURRENT_MA;
      contact_high_ticks = current_high ? contact_high_ticks + 1 : 0;
      placing_contact = distance_close || contact_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (placing_contact) {
        // True stop -- doesn't open the claw. Dropping a pin can't be
        // undone, so that's still a deliberate, separate button press.
        motor.move(0);
        return;
      }
    } else {
      placing_contact = false;
      contact_high_ticks = 0;

      bool current_high = current_ma() > CEILING_CURRENT_MA;
      ceiling_high_ticks = current_high ? ceiling_high_ticks + 1 : 0;
      at_ceiling = ceiling_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (at_ceiling) {
        motor.move(0);
        return;
      }
    }

    motor.move(stick);
    return;
  }

  // Reset here too, not just above -- these used to only get cleared
  // inside the active-stick branch, so letting go of R1/R2 could leave
  // TOUCHED or CEILING showing on the screen long after it was true.
  placing_contact = false;
  at_ceiling = false;
  contact_high_ticks = 0;
  ceiling_high_ticks = 0;

  if (!have_reading) {
    // Nothing to measure against -- stop rather than guess.
    homing = false;
    holding = false;
    motor.move(0);
    return;
  }

  if (homing) {
    int error = target_mm - mm;
    if (std::abs(error) <= ARRIVED_TOLERANCE_MM) {
      homing = false;
      motor.move(0);
      return;
    }
    motor.move(seek_speed(error));
    return;
  }

  // Idle hold: capture wherever the lift is the instant both buttons are
  // let go, then mostly leave it to the motor's own brake mode -- only
  // step in once it's drifted past HOLD_DEADBAND_MM (see that constant).
  if (!holding) {
    holding = true;
    hold_target_mm = mm;
  }
  int drift = hold_target_mm - mm;
  if (std::abs(drift) > HOLD_DEADBAND_MM) {
    motor.move(seek_speed(drift));
  } else {
    motor.move(0);
  }
}

}  // namespace lift
