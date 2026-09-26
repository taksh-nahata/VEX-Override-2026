// Doesn't touch EZ-Template/chassis, so skip main.h's heavy include chain
// (drive.hpp, <bits/stdc++.h> via EZ-Template/util.hpp) to keep this file's
// compile time down.
#include "api.h"
#include "globals.hpp"
#include "subsystems/toggle.hpp"

namespace toggle {

pros::Motor spinner(PORT_TOGGLE_SPINNER, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Optical color_sensor(PORT_TOGGLE_COLOR_SENSOR);

// TODO(tune): these are guesses, not numbers we've actually checked
// against a real toggle under real field lighting.
constexpr double HUE_RED_MAX = 20.0;
constexpr double HUE_YELLOW_MIN = 45.0;
constexpr double HUE_YELLOW_MAX = 65.0;
constexpr double HUE_BLUE_MIN = 200.0;
constexpr double HUE_BLUE_MAX = 250.0;

// Which color the driver wants the toggle spun to. We made this a
// controller button at match time instead of hardcoding it, since which
// color is "ours" depends on which alliance we're on that match.
Color target = Color::RED;

void initialize() {
  color_sensor.set_led_pwm(100);
}

void spin(int voltage) {
  spinner.move(voltage);
}

Color detect() {
  double hue = color_sensor.get_hue();
  if (hue <= HUE_RED_MAX) return Color::RED;
  if (hue >= HUE_YELLOW_MIN && hue <= HUE_YELLOW_MAX) return Color::YELLOW;
  if (hue >= HUE_BLUE_MIN && hue <= HUE_BLUE_MAX) return Color::BLUE;
  return Color::NONE;
}

Color target_color() {
  return target;
}

// Flips between red and blue. From yellow it lands on red -- yellow gets
// its own button (set_target_yellow()) since it isn't part of the
// alliance-color swap.
void toggle_target_red_blue() {
  target = (target == Color::BLUE) ? Color::RED : Color::BLUE;
}

void set_target_yellow() {
  target = Color::YELLOW;
}

const char* color_name(Color c) {
  switch (c) {
    case Color::RED:
      return "RED";
    case Color::BLUE:
      return "BLUE";
    case Color::YELLOW:
      return "YELLOW";
    default:
      return "NONE";
  }
}

}  // namespace toggle
