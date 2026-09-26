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

// Cancels manual control and PIDs down to position 0. Not currently wired
// to a button.
void go_to_floor();

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
