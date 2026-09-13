#pragma once

// DR4B lift: 1 motor per side, held level with a synced dual-motor PID.
//
// Override only lets a robot carry 1 pin + 1 cup at a time (no accumulator
// stacking), so the height needed each cycle depends on however tall the
// goal's stack has grown so far, not a fixed preset. Manual proportional
// control lines it up with whatever height that is; go_to_floor() is a
// macro for the one height that IS fixed and used every cycle (intake).
namespace lift {

void initialize();

// Current average height (motor degrees).
double position();

// Drives the lift directly from a joystick axis (-127 to 127), correcting
// for left/right skew. Call every opcontrol loop, even when centered.
void update(int stick);

// Cancels manual control and PIDs back to the floor/intake height.
void go_to_floor();

}  // namespace lift
