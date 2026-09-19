#pragma once

#include <cstdint>

// DR4B lift: 1 motor per side, held level with a synced dual-motor PID.
//
// Override only lets a robot carry 1 pin + 1 cup at a time (no accumulator
// stacking), so the height needed each cycle depends on however tall the
// goal's stack has grown so far, not a fixed preset. Manual proportional
// control lines it up with whatever height that is; go_to_floor() is a
// macro for the one height that IS fixed and used every cycle (intake).
//
// All tunable constants live in lift.cpp, tagged TODO(tune) — grep there
// for the full list of what still needs a real-world number.
namespace lift {

void initialize();

// Current average height (motor degrees).
double position();

// Drives the lift directly from R1 (+127) / R2 (-127) / neither (0),
// correcting for left/right skew. Call every opcontrol loop, even when
// neither button is held.
void update(int stick);

// Cancels manual control and PIDs back to the floor/intake height. Not
// currently wired to any button.
void go_to_floor();

// For tuning CONTACT_CURRENT_MA/CEILING_CURRENT_MA (lift.cpp) — print
// these while manually lowering onto a real stack, or raising to the
// mechanical top, to see normal load vs. the spike on contact.
std::int32_t left_current_ma();
std::int32_t right_current_ma();

// True right after update() stops a downward move due to a current spike
// (a pin landing on something). Independent of the floor limit below —
// one's current-based, the other's position-based.
bool touched_down();

// True right after update() stops an upward move due to a current spike
// (hit the mechanical ceiling). Fully automatic, no calibration needed —
// see lift.cpp for why the ceiling doesn't need the floor's manual toggle.
bool at_ceiling_now();

// Toggles the floor limit on/off (see main.cpp for the button). Re-enabling
// captures wherever the lift currently is as the new floor reference.
void toggle_floor_limit();
bool floor_limit_on();

}  // namespace lift
