// Only needs ez::PID, not the rest of EZ-Template (chassis/drive.hpp is by
// far the biggest header in this project) or main.h's other includes, so
// pull in just what's used to keep this file's compile time down.
#include "api.h"
#include "EZ-Template/PID.hpp"
#include "globals.hpp"
#include "subsystems/lift.hpp"
#include <cmath>
#include <cstdint>

namespace lift {

// ============================================================================
// HARDWARE
// 1 motor on the second four-bar, 1:6 external reduction (12T motor / 72T
// four-bar shaft). position() reads a rotation sensor on the 72T shaft
// (true arm angle, past the gear mesh) instead of the motor's own encoder.
// The motor is still what's driven and what current_ma() reads.
// ============================================================================
pros::Motor motor(PORT_LIFT, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Rotation rotation(PORT_LIFT_ROTATION);

// ============================================================================
// TUNABLE CONSTANTS — grep "TODO(tune)"/"TODO(verify)" for what still needs
// a real-world number.
// ============================================================================

// TODO(tune): retune for the 1:6 single-motor reduction.
ez::PID height_pid(0.4, 0.0, 1.0, 0);

// TODO(tune): feedforward push against gravity, added on top of the PID
// output so the PID only has to correct leftover error, not fight a known
// constant disturbance every tick.
constexpr int GRAVITY_HOLD = 15;

constexpr int STICK_DEADBAND = 10;

// TODO(tune): how close to 0 counts as "done homing" in go_to_floor().
constexpr double FLOOR_TOLERANCE_DEG = 10.0;

// TODO(tune): how far position() can drift from the idle-hold target
// before the PID corrects it. Below this, brake_mode HOLD (initialize())
// does the holding alone — keeps the lift resistant to a hand-push instead
// of the PID fighting every small nudge.
constexpr double HOLD_TOLERANCE_DEG = 5.0;

// TODO(tune)/TODO(verify): current (mA) that means "hit something solid."
// Ceiling's threshold is higher on purpose: raising fights gravity,
// lowering doesn't, so normal raising current runs higher than normal
// lowering current.
constexpr std::int32_t CONTACT_CURRENT_MA = 1500;
constexpr std::int32_t CEILING_CURRENT_MA = 2200;

// Consecutive ticks current must stay high before a contact stop fires —
// filters out the brief inrush spike every motor gives when it starts
// moving under load, so it doesn't read as a false contact.
constexpr int CONTACT_DEBOUNCE_TICKS = 5;

// ============================================================================
// STATE
// ============================================================================
bool homing = false;
bool holding = false;           // idle-hold target has been captured
double hold_target = 0;         // degrees, captured when idle hold engages
bool placing_contact = false;   // current-based stop fired while lowering
bool at_ceiling = false;        // current-based stop fired while raising
int contact_high_ticks = 0;
int ceiling_high_ticks = 0;

// ============================================================================
// QUERIES
// ============================================================================

void initialize() {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotation.reset_position();  // zeros to wherever the lift is right now —
                               // power on with it at true floor every time
}

double position() {
  return rotation.get_position() / 100.0;  // centidegrees -> degrees
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

void go_to_floor() {
  homing = true;
  holding = false;  // homing owns height_pid's target until it finishes
  height_pid.target_set(0);
}

void update(int stick) {
  if (std::abs(stick) > STICK_DEADBAND) {
    homing = false;
    holding = false;  // re-capture a fresh hold target next time it idles

    if (stick < 0) {
      at_ceiling = false;
      ceiling_high_ticks = 0;

      bool current_high = current_ma() > CONTACT_CURRENT_MA;
      contact_high_ticks = current_high ? contact_high_ticks + 1 : 0;
      placing_contact = contact_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (placing_contact) {
        motor.move(0);  // true stop -- does NOT open the claw, that's separate
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

  // Neither button held -- reset here too (not just above), or the screen
  // could keep showing TOUCHED/CEILING after you'd already let go.
  placing_contact = false;
  at_ceiling = false;
  contact_high_ticks = 0;
  ceiling_high_ticks = 0;

  if (homing) {
    double out = height_pid.compute(position()) + GRAVITY_HOLD;
    motor.move(out);
    if (std::fabs(position()) < FLOOR_TOLERANCE_DEG) homing = false;
    return;
  }

  // Idle hold: mostly brake_mode HOLD: PID only takes over past
  // HOLD_TOLERANCE_DEG of drift (gravity sag), see constant above for why.
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
