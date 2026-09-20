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

// ============================================================================
// HARDWARE
// ============================================================================
pros::Motor left_motor(PORT_LIFT_L, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor right_motor(PORT_LIFT_R, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// ============================================================================
// TUNABLE CONSTANTS
// Every one of these is a placeholder guess until tagged otherwise. Grep
// "TODO(tune)" for the full list of what still needs a real-world number.
// ============================================================================

// TODO(tune): retune once the DR4B is built, weighed, and the one-tooth
// gear issue is physically fixed (see git history 2026-09-18) — tuning
// against a mechanically crooked lift will just bake that crookedness into
// the gains.
ez::PID height_pid(0.4, 0.0, 1.0, 0);
ez::PID sync_pid(0.2, 0.0, 0.0, 0);

// TODO(tune): smallest constant that stops the lift sagging under gravity
// while height_pid is holding — too much fights the driver lowering it and
// adds to the current draw the contact-detection below reacts to. Gravity
// feedforward: a constant push added on top of the PID output so the PID
// only has to correct leftover error instead of fighting a known, constant
// disturbance every tick — see
// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html.
// A DR4B isn't a simple elevator (constant kG) or single-jointed arm
// (kCos * cos(angle)) — real holding torque varies through the four-bar's
// stroke — but a flat constant is a reasonable starting point. Only feeds
// into the homing/PID branch below, NOT the idle brake-hold (see there).
constexpr int GRAVITY_HOLD = 15;

constexpr int STICK_DEADBAND = 10;
constexpr double FLOOR_TOLERANCE_DEG = 10.0;

// TODO(tune): smallest value that still keeps both sides level. Deliberately
// capped — see sync_correction() below for why an uncapped value is a bug,
// not just imprecise.
constexpr double MAX_CORRECTION = 30;

// TODO(tune): current (mA) while LOWERING that means a pin has landed on
// something solid. TODO(verify): watch left_current_ma()/right_current_ma()
// on the debug screen while manually lowering onto a real stack to find
// normal descending load vs. the spike on actual contact — this number is
// a total guess right now, and will read falsely high until the one-tooth
// gear issue is fixed and the top stage is fully rubber-banded (both add
// current draw that has nothing to do with contact).
constexpr std::int32_t CONTACT_CURRENT_MA = 1500;

// TODO(tune): current (mA) while RAISING that means the lift has hit its
// own mechanical ceiling and the gear cartridge is skipping.
//
// Bumped 2026-09-20 from the old 1500 (copy-pasted from CONTACT_CURRENT_MA,
// never actually measured for this direction) — raising fights gravity while
// lowering has gravity helping, so normal *fine* raising current sits well
// above what's normal for lowering, and 1500 was getting crossed constantly
// during ordinary raising, not just at a real ceiling. That's what was
// actually causing the "glitches / one side at a time / slower" going up:
// at_ceiling kept false-triggering and stopping/restarting, and the
// one-tooth-off gear (still not physically fixed) made the two sides cross
// that false threshold at slightly different times, reading as one side
// stalling while the other kept moving.
//
// 2200 is still just a safer guess, not a measured value — read
// left_current_ma()/right_current_ma() off debug screen line 1 while
// raising normally (no glitching) and note the highest steady number you
// see, then again right as it actually skips/grinds at the true top, and
// send both so this can be set to something real in between.
constexpr std::int32_t CEILING_CURRENT_MA = 2200;

// TODO(tune): how many consecutive ticks (~20ms each) current has to stay
// above threshold before either contact check actually fires. Motors draw
// a brief inrush current spike just from starting to move under load —
// without this, a single-tick reading right as R1/R2 is first pressed
// could exceed the threshold, stop the motors, current drops since
// they're stopped, then the very next tick it tries again and spikes
// again — a rapid stop-start stutter ("glitching") instead of a clean
// contact stop. Requiring a few consecutive high readings filters that
// out while still catching a genuine sustained stall.
constexpr int CONTACT_DEBOUNCE_TICKS = 5;

// ============================================================================
// STATE
// ============================================================================
bool homing = false;
bool placing_contact = false;  // true right after a lowering move stopped on contact (see touched_down())
bool at_ceiling = false;       // true right after a raising move stopped on contact (see at_ceiling_now())
int contact_high_ticks = 0;    // consecutive ticks current has read high while lowering
int ceiling_high_ticks = 0;    // consecutive ticks current has read high while raising
bool floor_limit_enabled = true;
double floor_reference = 0;  // updated to "here" each time the floor limit is re-enabled

// ============================================================================
// QUERIES
// ============================================================================

void initialize() {
  left_motor.tare_position();
  right_motor.tare_position();
  left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

double position() {
  return (left_motor.get_position() + right_motor.get_position()) / 2.0;
}

double left_position() {
  return left_motor.get_position();
}

double right_position() {
  return right_motor.get_position();
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

// Toggle so it can be switched off while troubleshooting (e.g. the
// crooked-lift issue) without editing code, and back on afterward. Turning
// it back ON captures wherever the lift is AT THAT MOMENT as the new floor
// — not the original boot position — so you can disable it, reposition,
// and re-enable to set a new floor on the fly. See main.cpp for the button.
void toggle_floor_limit() {
  floor_limit_enabled = !floor_limit_enabled;
  if (floor_limit_enabled) floor_reference = lowest_position();
}

bool floor_limit_on() {
  return floor_limit_enabled;
}

// ============================================================================
// SYNC CORRECTION
// ============================================================================

// Keeps both sides level regardless of who's driving the lift (manual or
// PID). Capped so it can only ever nudge, never override — an uncapped
// correction grows with however out-of-sync the sides currently are, and
// once it's bigger than the commanded stick value it flips that side's
// sign entirely: e.g. holding R1 (both sides should rise) but a large
// correction pushes stick - correction negative, so the left side drops
// instead. That was the "R1/R2 sometimes goes the wrong way" bug (fixed
// 2026-09-18) — it got worse the more the two sides had drifted apart
// since boot, which is why it was intermittent rather than every time.
//
// TODO(mechanical, tracked externally not here): one lift motor's gear is
// seated one tooth off, which is a fixed mechanical disagreement this PID
// can never actually resolve — it will keep straining against an
// unreachable target until that gear is physically reseated. Expect
// elevated current and possibly false TOUCHED/CEILING readings until then.
double sync_correction() {
  double skew = left_motor.get_position() - right_motor.get_position();
  double correction = sync_pid.compute_error(-skew, skew);
  return std::clamp(correction, -MAX_CORRECTION, MAX_CORRECTION);
}

// ============================================================================
// PUBLIC CONTROL
// ============================================================================

// Cancels manual control and PIDs back to the floor/intake height. Not
// currently wired to any button (see main.cpp) — say so if you want it
// back, e.g. to actually exercise GRAVITY_HOLD above.
void go_to_floor() {
  homing = true;
  height_pid.target_set(0);
}

// Drives the lift directly from R1 (+127) / R2 (-127) / neither (0).
// Call every opcontrol loop, even when neither is held.
void update(int stick) {
  double correction = sync_correction();

  if (std::abs(stick) > STICK_DEADBAND) {
    homing = false;

    if (stick < 0) {
      at_ceiling = false;
      ceiling_high_ticks = 0;

      // Floor: never drive below floor_reference — a true stop, not just
      // zeroing the driver's input; correction doesn't get to sneak the
      // motors past this either.
      if (floor_limit_enabled && lowest_position() <= floor_reference) {
        placing_contact = false;
        contact_high_ticks = 0;
        left_motor.move(0);
        right_motor.move(0);
        return;
      }

      // Placing: stop lowering once something solid is under the pin for
      // several consecutive ticks in a row (see CONTACT_DEBOUNCE_TICKS),
      // instead of continuing to grind into it. Also a true stop. Does NOT
      // open the claw — release stays a deliberate, separate button press,
      // since it's the one irreversible step here (can't un-drop a pin)
      // and this detection is still unverified.
      bool current_high = left_current_ma() > CONTACT_CURRENT_MA || right_current_ma() > CONTACT_CURRENT_MA;
      contact_high_ticks = current_high ? contact_high_ticks + 1 : 0;
      placing_contact = contact_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (placing_contact) {
        left_motor.move(0);
        right_motor.move(0);
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
      bool current_high = left_current_ma() > CEILING_CURRENT_MA || right_current_ma() > CEILING_CURRENT_MA;
      ceiling_high_ticks = current_high ? ceiling_high_ticks + 1 : 0;
      at_ceiling = ceiling_high_ticks >= CONTACT_DEBOUNCE_TICKS;
      if (at_ceiling) {
        left_motor.move(0);
        right_motor.move(0);
        return;
      }
    }

    left_motor.move(stick - correction);
    right_motor.move(stick + correction);
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
    left_motor.move(out - correction);
    right_motor.move(out + correction);
    if (std::fabs(position()) < FLOOR_TOLERANCE_DEG) homing = false;
    return;
  }

  // Idle hold: brake_mode HOLD (set in initialize()) is a real closed-loop
  // mechanism at the motor firmware level — move(0) here engages it, it
  // doesn't just coast. Don't add GRAVITY_HOLD or any other constant on
  // top of this; that would override the firmware's own feedback with a
  // cruder open-loop guess instead of complementing it.
  left_motor.move(0);
  right_motor.move(0);
}

}  // namespace lift
