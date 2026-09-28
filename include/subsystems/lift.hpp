#pragma once

#include <cstdint>

// DR4B lift: 1 motor on the second four-bar, through a 1:6 external
// reduction. Height control is entirely driven by the claw's distance
// sensor (points down) -- there's no separate degree/rotation-sensor
// math involved for real moves. Everything here is in INCHES, to match
// the rest of the project (drivetrain, odometry, autons). In practice
// this sensor mostly reads distance to the floor, not necessarily
// whatever's stacked on a goal, so every target below is really "how
// high above the ground," not "how far above the current stack." See
// lift.cpp.
namespace lift {

void initialize();

// Height, in degrees, from the rotation sensor. Only used by things
// OUTSIDE the lift itself (main.cpp's anti-tip, to scale speed by how
// high the lift is) -- the lift's own control doesn't use this for
// real moves anymore (it does still use it internally for idle hold --
// see lift.cpp).
double position();

// Drives the lift from R1 (+127) / R2 (-127) / neither (0). Call every
// opcontrol loop, even when neither button is held.
void update(int stick);

// The one real move: rises/lowers until the claw distance sensor reads
// target_in (inches). Use this directly in autons.cpp for a height that
// doesn't have its own name below. If the sensor can't see anything,
// the lift just doesn't move rather than guessing.
void go_to_height(double target_in);

// Goes to true floor (go_to_height(0)).
void go_to_floor();

// One button per Goal type (Alliance/Neutral/Center all have different
// heights -- game manual Appendix B) -- the height to place a first pin
// onto an empty Goal of that type. Bound to X/B/A in main.cpp right now
// for bench testing. Numbers are guesses in lift.cpp, tagged TODO(tune).
void go_to_alliance_goal();
void go_to_neutral_goal();
void go_to_center_goal();

// The old pin-count presets (1st/2nd/3rd pin stacked on whatever Goal
// you're already at) -- still used by auton_button_2()'s Loader cycles.
void go_to_pin_1();
void go_to_pin_2();
void go_to_pin_3();

// Just enough to clear the ground and hold the preload pin at
// cup-opening height while driving to drop it in. Used by
// auton_button_1() (autons.cpp).
void go_to_cup_drop();

// Places a whole nested Cup+Pin unit onto the goal -- taller than a
// bare Pin, so it uses its own height. Used by auton_button_1().
void go_to_pin_on_cup();

// True while a go_to_*() move is still in progress. The lift only
// actually moves while something calls update() -- unlike the
// chassis's own PID, it doesn't run on a background task -- so auton
// code has to poll this and keep calling update(0) itself while
// waiting.
bool is_homing();

// Motor current draw (mA) — for tuning CONTACT_CURRENT_MA/CEILING_CURRENT_MA
// (lift.cpp) against the debug screen.
std::int32_t current_ma();

// Inches from the claw down to whatever's under it (port 6 distance
// sensor). A very large number (roughly 390in+) means it can't see
// anything solid. For the debug screen, and for tuning the height
// presets (lift.cpp) against a ruler
double claw_distance_in();

// True for the tick(s) right after update() stops a downward move --
// either the claw distance sensor read close enough to count as arrived
// (stops it gently, before contact) or, as a backup, current spiked
// (actual contact -- catches it even if the distance reading was bad).
bool touched_down();

// True for the tick(s) right after update() stops an upward move because
// current spiked — hit the mechanical ceiling.
bool at_ceiling_now();

}  // namespace lift
