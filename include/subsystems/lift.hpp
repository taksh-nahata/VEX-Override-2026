#pragma once

#include <cstdint>

// DR4B lift: 1 motor on the second four-bar, through a 1:6 external
// reduction. Height control is entirely driven by the claw's distance
// sensor (points down, sees whatever's under the claw) -- there's no
// separate degree/rotation-sensor math involved. See lift.cpp.
namespace lift {

void initialize();

// Height, in degrees, from the rotation sensor. Only used by things
// OUTSIDE the lift itself (main.cpp's anti-tip, to scale speed by how
// high the lift is) -- the lift's own control doesn't use this anymore.
double position();

// Drives the lift from R1 (+127) / R2 (-127) / neither (0). Call every
// opcontrol loop, even when neither button is held.
void update(int stick);

// Goes to true floor (clearance 0 above whatever's under the claw).
void go_to_floor();

// Presets for the three heights we actually use in a match: the pin
// going on an empty goal, on a goal with 1 pin already on it, and on a
// goal with 2. Bound to X/B/A in main.cpp. Each one reads the claw
// distance sensor to see how far it is from whatever's under it right
// now, and rises that many pins' worth higher -- so the same button
// works whether the goal's empty or already has pins on it. If the
// sensor can't see anything, the lift just doesn't move rather than
// guessing. Tunable numbers are in lift.cpp, tagged TODO(tune).
void go_to_pin_1();
void go_to_pin_2();
void go_to_pin_3();

// Just enough to clear the ground and hold the preload pin at
// cup-opening height while driving to drop it in. Used by
// auton_button_1() (autons.cpp).
void go_to_cup_drop();

// Places a whole nested Cup+Pin unit onto the goal -- taller than a
// bare Pin, so it uses its own clearance number. Used by
// auton_button_1().
void go_to_cup_on_goal();

// True while a go_to_*() move is still in progress. The lift only
// actually moves while something calls update() -- unlike the
// chassis's own PID, it doesn't run on a background task -- so auton
// code has to poll this and keep calling update(0) itself while
// waiting.
bool is_homing();

// Motor current draw (mA) — for tuning CONTACT_CURRENT_MA/CEILING_CURRENT_MA
// (lift.cpp) against the debug screen.
std::int32_t current_ma();

// mm from the claw down to whatever's under it (port 6 distance sensor).
// 9999 means it can't see anything solid. For the debug screen, and for
// tuning the clearance constants (lift.cpp) against a ruler.
std::int32_t claw_distance_mm();

// True for the tick(s) right after update() stops a downward move --
// either the claw distance sensor read close enough to count as arrived
// (stops it gently, before contact) or, as a backup, current spiked
// (actual contact -- catches it even if the distance reading was bad).
bool touched_down();

// True for the tick(s) right after update() stops an upward move because
// current spiked — hit the mechanical ceiling.
bool at_ceiling_now();

}  // namespace lift
