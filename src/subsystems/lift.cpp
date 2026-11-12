#include "api.h"
#include "globals.hpp"
#include "subsystems/lift.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lift {

// two motors on one shaft (MotorGroup gave a link error)
pros::Motor motor(PORT_LIFT, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor motor_2(PORT_LIFT_2, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Rotation rotation(PORT_LIFT_ROTATION);  // for position()
pros::Distance claw_distance(PORT_CLAW_DISTANCE);  // height comes from this

constexpr double SEEK_GAIN = 15.0;    // speed per inch of error
constexpr int SEEK_SPEED = 100;       // max speed going to a height
constexpr int GRAVITY_HOLD = 15;      // extra push going up
constexpr double TOLERANCE_IN = 0.4;  // close enough

bool homing = false;
double target_in = 0;


void initialize() {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  motor_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

constexpr double LIFT_BOTTOM_DEG = 244.2;  // raw reading with the lift all the way down

double raw_angle() {
  return rotation.get_angle() / 100.0;
}

double position() {
  double deg = LIFT_BOTTOM_DEG - raw_angle();  // raw goes down as the lift goes up
  if (deg > 180) deg -= 360;  // in case 0/360 is inside the lift's travel
  if (deg < -180) deg += 360;
  return deg;
}

// limits in position() degrees, everything goes through move()
constexpr double LIFT_MIN_DEG = 0;
constexpr double LIFT_MAX_DEG = 100;
// slows down near the limits so momentum doesn't carry it past
constexpr double SLOW_ZONE_DEG = 30;
constexpr int CREEP = 25;

void move(int voltage) {
  double raw = raw_angle();
  if (raw <= 360) {  // a failed read is huge, skip the limits then
    double pos = position();
    double left = voltage > 0 ? LIFT_MAX_DEG - pos : pos - LIFT_MIN_DEG;  // degrees to the limit ahead
    if (voltage != 0) {
      if (left <= 0) voltage = 0;
      else if (left < SLOW_ZONE_DEG) {
        int cap = std::max(CREEP, static_cast<int>(127 * left / SLOW_ZONE_DEG));
        voltage = std::clamp(voltage, -cap, cap);
      }
    }
  }
  motor.move(voltage);
  motor_2.move(voltage);
}

std::int32_t current_ma() {
  return motor.get_current_draw();
}

double claw_distance_in() {
  return claw_distance.get() / 25.4;
}

bool is_homing() {
  return homing;
}

void go_to_height(double inches) {
  target_in = inches;
  homing = true;
}

void go_to_cup_and_pin() { go_to_height(4.5); }

void update(int stick) {
  int mm = claw_distance.get();
  bool sees_something = mm > 0 && mm < 9999;  // 9999 = sees nothing
  double height_in = mm / 25.4;

  if (std::abs(stick) > 10) {  // manual control cancels the preset
    homing = false;
    bool on_floor = sees_something && height_in <= TOLERANCE_IN;
    move(stick < 0 && on_floor ? 0 : stick);
    return;
  }

  if (homing && sees_something) {
    double error = target_in - height_in;
    if (std::fabs(error) > TOLERANCE_IN) {
      int speed = std::clamp(static_cast<int>(error * SEEK_GAIN), -SEEK_SPEED, SEEK_SPEED);
      move(speed > 0 ? speed + GRAVITY_HOLD : speed);
      return;
    }
  }

  homing = false;
  move(0);  // hold brake keeps it up
}

}  // namespace lift
