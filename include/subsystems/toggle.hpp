#pragma once

// Toggle spinner motor + optical sensor for reading the toggle's set color.
namespace toggle {

enum class Color { NONE, RED, BLUE, YELLOW };

void initialize();
void spin(int voltage);
Color detect();

// Which color the spinner is currently trying to reach (see main.cpp for
// the buttons that change this). Defaults to RED.
Color target_color();

// Flips the target between red/blue (from yellow, goes to red).
void toggle_target_red_blue();

// Sets the target directly to yellow.
void set_target_yellow();

// "RED"/"BLUE"/"YELLOW"/"NONE" — for printing to the screen or controller.
const char* color_name(Color c);

}  // namespace toggle
