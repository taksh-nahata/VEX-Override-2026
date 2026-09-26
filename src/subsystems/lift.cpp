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
// Rebuilt 2026-09-20 for the single-motor DR4B (was 2 motors with a
// dual-side synced PID) — see globals.hpp's PORT_LIFT comment for why.
//
// position() reads the rotation sensor, not the motor's own encoder — it's
// mounted on the far side of the 12T:72T external reduction (on the
// four-bar's actual shaft), so it reports true arm angle directly instead
// of motor-shaft rotation that assumes a perfect, backlash-free gear mesh.
// The motor is still what's driven (move()) and still what current_ma()
// reads — the rotation sensor has no current draw, it's a pure angle
// sensor.
// ============================================================================
pros::Motor motor(PORT_LIFT, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Rotation rotation(PORT_LIFT_ROTATION);

// ============================================================================
// TUNABLE CONSTANTS
// Every one of these is a placeholder guess until tagged otherwise. Grep
// "TODO(tune)" for the full list of what still needs a real-world number.
// ============================================================================

// TODO(tune): retune for the new single-motor/1:6 external reduction — the
// old gains were never trustworthy anyway (tuned against a mechanically
// crooked 2-motor lift, see git history 2026-09-18/20).
ez::PID height_pid(0.4, 0.0, 1.0, 0);

// TODO(tune): smallest constant that stops the lift sagging under gravity
// while height_pid is holding — too much fights the driver lowering it and
// adds to the current draw the contact-detection below reacts to. Gravity
// feedforward: a constant push added on top of the PID output so the PID
// only has to correct leftover error instead of fighting a known, constant
// disturbance every tick — see
// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html.
// A DR4B isn't a simple elevator (constant kG) or single-jointed arm
// (kCos * cos(angle)) — real holding torque varies through the four-bar's
// stroke — but a flat constant is a reasonable starting point. Feeds into
// both PID branches below (homing and idle hold) — anywhere height_pid
// runs, gravity's fighting it the same way.
constexpr int GRAVITY_HOLD = 15;

constexpr int STICK_DEADBAND = 10;
// TODO(tune): position() reads a rotation sensor past the new 1:6 external
// reduction as of 2026-09-20, not a motor's own encoder — a "degree" here
// covers ~6x the arm movement a raw motor degree used to, so this
// unchanged-since-the-2-motor-days number is probably too loose now.
constexpr double FLOOR_TOLERANCE_DEG = 10.0;

// TODO(tune): current (mA) while LOWERING that means a pin has landed on
// something solid. TODO(verify): watch current_ma() on the debug screen
// while manually lowering onto a real stack to find normal descending load
// vs. the spike on actual contact — this number is a total guess right now.
constexpr std::int32_t CONTACT_CURRENT_MA = 1500;

// TODO(tune): current (mA) while RAISING that means the lift has hit its
// own mechanical ceiling. Still an unmeasured guess — read current_ma() off
// debug screen line 1 while raising normally (steady-state, no glitching)
// and note the highest number you see, then again right at the true top,
// and send both so this can be set to something real in between. Kept
// higher than CONTACT_CURRENT_MA on purpose: raising fights gravity,
// lowering has gravity helping, so normal *fine* raising current sits
// above normal lowering current — see git history 2026-09-20 for how using
// the same number for both caused false-triggering on the old 2-motor lift.
constexpr std::int32_t CEILING_CURRENT_MA = 2200;

// TODO(tune): how many consecutive ticks (~20ms each) current has to stay
// above threshold before either contact check actually fires. Motors draw
// a brief inrush current spike just from starting to move under load —
// without this, a single-tick reading right as R1/R2 is first pressed
// could exceed the threshold, stop the motor, current drops since it's
// stopped, then the very next tick it tries again and spikes again — a
// rapid stop-start stutter ("glitching") instead of a clean contact stop.
// Requiring a few consecutive high readings filters that out while still
// catching a genuine sustained stall.
constexpr int CONTACT_DEBOUNCE_TICKS = 5;

// ============================================================================
// STATE
// ============================================================================
bool homing = false;
bool holding = false;          // true once an idle hold target has been captured (see update())
bool placing_contact = false;  // true right after a lowering move stopped on contact (see touched_down())
bool at_ceiling = false;       // true right after a raising move stopped on contact (see at_ceiling_now())
int contact_high_ticks = 0;    // consecutive ticks current has read high while lowering
int ceiling_high_ticks = 0;    // consecutive ticks current has read high while raising
bool floor_limit_enabled = true;
double floor_reference = 0;  // updated to "here" each time the floor limit is re-enabled

// ============================================================================
// QUERIES
// ============================================================================

// IMPORTANT: this zeros position() to wherever the lift physically is
// right now, not to any true fixed reference — the rotation sensor has no
// memory of "true floor" across a power cycle, same as the motor's own
// encoder wouldn't. Everything downstream (the floor limit, and
// MAX_LIFT_HEIGHT_DEG in main.cpp) assumes 0 means true floor, so the team
// has to physically power on with the lift all the way down every time.
// Cheap fix, no code — see TODO.md.
void initialize() {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotation.reset_position();
}

// Degrees, from the rotation sensor (see HARDWARE comment above for why
// not the motor's own encoder). Rotation reports centidegrees.
double position() {
  return rotation.get_position() / 100.0;
}

std::int32_t current_ma() {
  return motor.get_current_draw();
}

// True for the tick(s) after update() last stopped a LOWERING move because
// of a current spike (i.e. it thinks a pin just landed on something). NOT
// the same thing as the floor limit — this is current-based, the floor
// limit is position-based, they're independent checks.
bool touched_down() {
  return placing_contact;
}

// True for the tick(s) after update() last stopped a RAISING move because
// of a current spike (i.e. it thinks the lift hit its mechanical ceiling
// and would otherwise skip). Fully automatic — no manual calibration step,
// unlike the floor limit, because there's always something to hit at the
// true mechanical top, the same reasoning that makes touched_down() sound
// for lowering onto a stack.
bool at_ceiling_now() {
  return at_ceiling;
}

// ============================================================================
// FLOOR LIMIT
// Position-based (not current-based like the ceiling) because "0" has a
// natural, known-good reference: wherever the lift was at boot. The
// ceiling doesn't get the same treatment because we have no equivalent
// known-good reference for the top — current-sensing fills that gap
// instead of needing one.
// ============================================================================

// Toggle so it can be switched off while troubleshooting without editing
// code, and back on afterward. Turning it back ON captures wherever the
// lift is AT THAT MOMENT as the new floor — not the original boot position
// — so you can disable it, reposition, and re-enable to set a new floor on
// the fly. See main.cpp for the button.
void toggle_floor_limit() {
  floor_limit_enabled = !floor_limit_enabled;
  if (floor_limit_enabled) floor_reference = position();
}

bool floor_limit_on() {
  return floor_limit_enabled;
}

// ============================================================================
// PUBLIC CONTROL
// ============================================================================

// Cancels manual control and PIDs back to the floor/intake height. Not
// currently wired to any button (see main.cpp) — say so if you want it
// back, e.g. to actually exercise GRAVITY_HOLD above.
void go_to_floor() {
  homing = true;
  holding = false;  // homing owns height_pid's target until it finishes
  height_pid.target_set(0);
}

// Drives the lift directly from R1 (+127) / R2 (-127) / neither (0).
// Call every opcontrol loop, even when neither is held.
void update(int stick) {
  if (std::abs(stick) > STICK_DEADBAND) {
    homing = false;
    holding = false;  // re-capture a fresh hold target next time it idles

    if (stick < 0) {
      at_ceiling = false;
      ceiling_high_ticks = 0;

      // Floor: never drive below floor_reference — a true stop, not just
      // zeroing the driver's input.
      if (floor_limit_enabled && position() <= floor_reference) {
        placing_contact = false;
        contact_high_ticks = 0;
        motor.move(0);
        return;
      }

      // Placing: stop lowering once something solid is under the pin for
      // several consecutive ticks in a row (see CONTACT_DEBOUNCE_TICKS),
      // instead of continuing to grind into it. Also a true stop. Does NOT
      // open the claw — release stays a deliberate, separate button press,
      // since it's the one irreversible step here (can't un-drop a pin)
      // and this detection is still unverified.
      bool current_high = current_ma() > CONTACT_CURRENT_MA;
      contact_high_ticks = current_high ? contact_high_ticks + 1 : 0;
      placing_contact = contact_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (placing_contact) {
        motor.move(0);
        return;
      }
    } else {
      placing_contact = false;
      contact_high_ticks = 0;

      // Ceiling: stop raising once the mechanism resists hard enough to
      // spike current for several consecutive ticks in a row — see
      // at_ceiling_now() above for why this doesn't need a manual
      // calibration step the way the floor did, and CONTACT_DEBOUNCE_TICKS
      // for why it's not a single-tick check.
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

  // Neither button held. Both flags (and their debounce counters) reset
  // here too, not just above — they used to only update inside the
  // active-stick branch, so the screen could keep showing TOUCHED/CEILING
  // long after you'd let go entirely.
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

  // Idle hold: actively servos against gravity instead of just relying on
  // brake_mode HOLD (still set in initialize(), but on its own it wasn't
  // enough — confirmed 2026-09-25, the single motor driving the whole DR4B
  // through the 1:6 reduction sags out of brake-hold alone). Captures
  // wherever the lift was the instant the stick let go as the target, then
  // uses the rotation sensor to correct back to it if it drifts — a real
  // closed loop, not a blind constant push. Re-captured fresh every time
  // (holding flips false in the stick branch above) so it always holds
  // "wherever you left it," not some stale height from earlier.
  if (!holding) {
    holding = true;
    height_pid.target_set(position());
  }
  double out = height_pid.compute(position()) + GRAVITY_HOLD;
  motor.move(out);
}

}  // namespace lift
