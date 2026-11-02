#pragma once

#include <cstdint>

// DR4B lift. Heights are inches from the claw's down-facing distance sensor.
// go_to_*() only sets a target -- update() must be called every loop to move.
namespace lift {

void initialize();

// Call every loop: stick = +127 (R1), -127 (R2) or 0. A stick press cancels any preset.
void update(int stick);

// Rises/lowers until the sensor reads this many inches. Does nothing if it sees nothing.
void go_to_height(double inches);

void go_to_cup_and_pin();  // button A

// True while a go_to_*() move is running (autons poll this, calling update(0)).
bool is_homing();

// For the debug screen / SD log / anti-tip.
double position();   // degrees above the bottom (rotation sensor, absolute)
double raw_angle();  // the sensor's own 0-360 reading, for measuring LIFT_BOTTOM_DEG
std::int32_t current_ma();
double claw_distance_in();

}  // namespace lift
