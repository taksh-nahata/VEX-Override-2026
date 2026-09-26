#pragma once

#include <cstdint>

// DR4B lift: 1 motor on the second four-bar, through a 1:6 external
// reduction. position() reads a rotation sensor mounted past that
// reduction, so it's telling us the four-bar's real angle instead of us
// inferring it from the motor's own encoder.
//
// Tunable constants and the reasoning behind them live in lift.cpp,
// tagged TODO(tune)/TODO(verify).
namespace lift {

void initialize();

// Height, in degrees, from the rotation sensor.
double position();

// Drives the lift from R1 (+127) / R2 (-127) / neither (0). Call every
// opcontrol loop, even when neither button is held.
void update(int stick);

// Cancels manual control and PIDs down to the floor (position 0). Wired
// to no button of its own -- it's what go_to_pin_1/2/3() below build on.
void go_to_floor();

// Presets for the three heights we actually use in a match: the pin
// that's going on an empty goal, on a goal with 1 pin already on it, and
// on a goal with 2. Bound to X/B/A in main.cpp -- meant to be something a
// driver can push without having to think about exact heights themselves.
// Target heights are guesses in lift.cpp, tagged TODO(tune) -- there's no
// substitute for testing these against the real stack heights.
void go_to_pin_1();
void go_to_pin_2();
void go_to_pin_3();

// True while a go_to_floor()/go_to_pin_1/2/3() move is still in progress.
// The lift only actually moves while something calls update() -- unlike
// the chassis's own PID, it doesn't run on a background task -- so auton
// code has to poll this and keep calling update(0) itself while waiting.
bool is_homing();

// Motor current draw (mA) — for tuning CONTACT_CURRENT_MA/CEILING_CURRENT_MA
// (lift.cpp) against the debug screen.
std::int32_t current_ma();

// True for the tick(s) right after update() stops a downward move because
// current spiked — hit something solid (a stack, or the true floor).
bool touched_down();

// True for the tick(s) right after update() stops an upward move because
// current spiked — hit the mechanical ceiling.
bool at_ceiling_now();

}  // namespace lift
