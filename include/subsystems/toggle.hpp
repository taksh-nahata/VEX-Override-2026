#pragma once

// Toggle spinner motor + optical sensor for reading the toggle's set color.
namespace toggle {

enum class Color { NONE, RED, BLUE, YELLOW };

void initialize();
void spin(int voltage);
Color detect();

}  // namespace toggle
