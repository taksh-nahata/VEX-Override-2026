#pragma once

#include <cstdint>

namespace lift {

void initialize();

void update(int stick);  // call every loop, stick is 127 (up), -127 (down) or 0
void go_to_height(double inches);  // only sets the target, update() does the moving
void go_to_cup_and_pin();          // button A
bool is_homing();                  // true while going to a height

double position();   // degrees above the bottom
double raw_angle();  // rotation sensor reading
std::int32_t current_ma();
double claw_distance_in();

}  // namespace lift
