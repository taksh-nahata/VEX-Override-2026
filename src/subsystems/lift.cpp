// Only needs ez::PID, not the rest of EZ-Template (chassis/drive.hpp is by
// far the biggest header in this project) or main.h's other includes, so
// we only pull in what this file actually uses to keep compile time down.
#include "api.h"
#include "EZ-Template/PID.hpp"
#include "globals.hpp"
#include "subsystems/lift.hpp"
#include <cmath>
#include <cstdint>

namespace lift {

// ============================================================================
// HARDWARE
// The lift used to be 2 motors, one per side, kept level with a synced
// PID. We switched to 1 motor on the second four-bar (1:6 external
// reduction, 12T on the motor to 72T on the four-bar's shaft) after a
// mismatched gear on one side kept causing current spikes no matter how
// we tuned the sync correction -- it was a fixed mechanical problem, not
// something a PID could fix. One motor removed the mismatch outright.
//
// position() reads a rotation sensor mounted on the 72T shaft, past the
// gear mesh, instead of the motor's own encoder -- that way it's reading
// the four-bar's actual angle, not just guessing at it from how far the
// motor thinks it turned. The motor itself is still what we drive and
// what current_ma() reads.
// ============================================================================
pros::Motor motor(PORT_LIFT, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Rotation rotation(PORT_LIFT_ROTATION);

// ============================================================================
// TUNABLE CONSTANTS — grep "TODO(tune)"/"TODO(verify)" for everything that
// still needs a real number off the actual robot.
// ============================================================================

// TODO(tune): needs retuning for the 1:6 single-motor setup -- these
// numbers are left over from the old 2-motor lift.
ez::PID height_pid(0.4, 0.0, 1.0, 0);

// TODO(tune): a constant push against gravity, added on top of whatever
// the PID computes, so the PID is only correcting leftover error instead
// of fighting the same predictable sag every single tick.
constexpr int GRAVITY_HOLD = 15;

constexpr int STICK_DEADBAND = 10;

// TODO(tune): how close counts as "arrived" for go_to_floor()/go_to_pin_1/2/3().
constexpr double HEIGHT_TOLERANCE_DEG = 10.0;

// TODO(tune): the three heights we actually need in a match -- the pin
// going onto an empty goal, a goal with 1 pin on it already, and a goal
// with 2. These are guesses spaced out across the still-unmeasured full
// range (MAX_LIFT_HEIGHT_DEG, main.cpp), not numbers we've checked against
// a real stack yet. Press the matching button, see how close it lands,
// adjust, rebuild -- same as every other number in this file.
constexpr double PIN_1_HEIGHT_DEG = 500;
constexpr double PIN_2_HEIGHT_DEG = 1000;
constexpr double PIN_3_HEIGHT_DEG = 1500;

// TODO(tune): how far the lift can sag from where it was left before the
// PID steps in to correct it. We added this after testing showed the
// first version of idle hold ran the PID every tick and made the lift
// noticeably easy to push by hand -- turns out the motor's own brake mode
// resists a hand-push a lot harder than our PID correcting once every
// ~20ms does, so below this we just let brake mode do the holding and
// only bring in the PID for the slow gravity sag it can't stop alone.
constexpr double HOLD_TOLERANCE_DEG = 5.0;

// TODO(tune)/TODO(verify): current (mA) that means the lift just hit
// something solid. The ceiling threshold is set higher on purpose --
// raising fights gravity and lowering doesn't, so normal raising current
// runs higher than normal lowering current even with nothing in the way.
constexpr std::int32_t CONTACT_CURRENT_MA = 1500;
constexpr std::int32_t CEILING_CURRENT_MA = 2200;

// How many ticks in a row current has to stay high before we actually
// call it a contact stop. Every motor gives a brief current spike just
// from starting to move under load, and without this debounce that spike
// alone would trip a false stop the instant R1/R2 is pressed.
constexpr int CONTACT_DEBOUNCE_TICKS = 5;

// ============================================================================
// STATE
// ============================================================================
bool homing = false;
bool holding = false;          // idle-hold target has been captured for this hold
double hold_target = 0;        // degrees, captured the instant idle hold engages
bool placing_contact = false;  // current-based stop fired while lowering
bool at_ceiling = false;       // current-based stop fired while raising
int contact_high_ticks = 0;
int ceiling_high_ticks = 0;

// ============================================================================
// QUERIES
// ============================================================================

void initialize() {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  // Zeros to wherever the lift physically is right now, not to any fixed
  // reference -- there's nothing in the hardware that remembers "true
  // floor" across a power cycle. So the team has to actually power on
  // with the lift all the way down every time, or this number means
  // something different depending on where it happened to be left.
  rotation.reset_position();
}

double position() {
  return rotation.get_position() / 100.0;  // sensor reports centidegrees
}

std::int32_t current_ma() {
  return motor.get_current_draw();
}

bool touched_down() {
  return placing_contact;
}

bool at_ceiling_now() {
  return at_ceiling;
}

// ============================================================================
// PUBLIC CONTROL
// ============================================================================

void go_to_height(double target_deg) {
  homing = true;
  holding = false;  // homing owns height_pid's target until it's done
  height_pid.target_set(target_deg);
}

void go_to_floor() {
  go_to_height(0);
}

void go_to_pin_1() {
  go_to_height(PIN_1_HEIGHT_DEG);
}

void go_to_pin_2() {
  go_to_height(PIN_2_HEIGHT_DEG);
}

void go_to_pin_3() {
  go_to_height(PIN_3_HEIGHT_DEG);
}

void update(int stick) {
  if (std::abs(stick) > STICK_DEADBAND) {
    homing = false;
    holding = false;  // let go of the old hold target -- we'll capture a new one next time it idles

    if (stick < 0) {
      at_ceiling = false;
      ceiling_high_ticks = 0;

      bool current_high = current_ma() > CONTACT_CURRENT_MA;
      contact_high_ticks = current_high ? contact_high_ticks + 1 : 0;
      placing_contact = contact_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (placing_contact) {
        // True stop -- doesn't open the claw. Dropping a pin can't be
        // undone, so we kept that a deliberate, separate button press
        // instead of tying it to a sensor reading we're still verifying.
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

  if (homing) {
    double out = height_pid.compute(position()) + GRAVITY_HOLD;
    motor.move(out);
    if (std::fabs(position() - height_pid.target_get()) < HEIGHT_TOLERANCE_DEG) homing = false;
    return;
  }

  // Idle hold: capture wherever the lift is the instant both buttons are
  // let go, then mostly leave it to the motor's own brake mode. The PID
  // only steps in once it's drifted past HOLD_TOLERANCE_DEG -- see that
  // constant for why we added the deadband instead of just running the
  // PID continuously.
  if (!holding) {
    holding = true;
    hold_target = position();
    height_pid.target_set(hold_target);
  }
  if (std::fabs(position() - hold_target) > HOLD_TOLERANCE_DEG) {
    motor.move(height_pid.compute(position()) + GRAVITY_HOLD);
  } else {
    motor.move(0);
  }
}

}  // namespace lift
