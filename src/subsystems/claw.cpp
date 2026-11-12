#include "api.h"
#include "globals.hpp"
#include "subsystems/claw.hpp"
#include <algorithm>
#include <cmath>

namespace claw {

pros::adi::DigitalOut piston(PORT_CLAW_SOLENOID);
pros::Distance pin_sensor(PORT_CLAW_PIN_DISTANCE);
bool is_closed = false;
volatile bool armed = false;
double (*pos_fn)() = nullptr;  // robot position in inches, set in main

constexpr double GRIP_MM = 43;           // sensor reading when the pin is where we want to grab it
constexpr double CLOSE_TIME_S = 0.1;     // how long the claw takes to close
constexpr double SENSOR_DELAY_S = 0.04;  // raise this if it still closes late
constexpr double LEAD_S = CLOSE_TIME_S + SENSOR_DELAY_S;

constexpr int LOOP_MS = 10;
constexpr double MAX_SPEED = 2500;  // anything faster is just EZ resetting the position
constexpr double SPEED_SMOOTH_S = 0.03, ACCEL_SMOOTH_S = 0.06;  // smoothing

// for the brain screen
volatile double shown_mm = 0, shown_speed = 0, shown_close = 0, shown_closed_mm = -1, shown_closed_speed = 0;

void open() {
  armed = false;
  piston.set_value(false);
  is_closed = false;
}

void close() {
  armed = false;
  piston.set_value(true);
  is_closed = true;
}

void close_when_pin() { armed = true; }

void set_pos_fn(double (*inches)()) { pos_fn = inches; }

// background task, closes the claw early based on how fast we're driving since the claw takes time to close
void initialize() {
  static pros::Task watcher([] {
    double speed = 0, accel = 0;  // mm/s, mm/s^2
    double last_pos = pos_fn ? pos_fn() * 25.4 : 0;
    double last_mm = -1;
    std::uint32_t last_t = pros::millis(), last_valid = 0;

    while (true) {
      std::uint32_t now = pros::millis();
      double dt = (now - last_t) / 1000.0;
      last_t = now;

      if (pos_fn && dt > 0) {
        double pos = pos_fn() * 25.4;
        double raw = (pos - last_pos) / dt;
        last_pos = pos;
        if (std::fabs(raw) <= MAX_SPEED) {
          double new_speed = speed + (1 - std::exp(-dt / SPEED_SMOOTH_S)) * (raw - speed);
          accel += (1 - std::exp(-dt / ACCEL_SMOOTH_S)) * ((new_speed - speed) / dt - accel);
          speed = new_speed;
        }
      }

      double v = std::max(speed, 0.0);  // only forward counts
      double travel = 0;  // how far we go while the claw closes
      if (v > 0) {
        bool stops = accel < 0 && v + accel * LEAD_S < 0;
        travel = stops ? v * v / (2 * -accel) : v * LEAD_S + 0.5 * accel * LEAD_S * LEAD_S;
        travel = std::clamp(travel, 0.0, 3 * v * LEAD_S);  // stops a tiny accel from blowing this up
      }
      double trigger = GRIP_MM + travel;

      int mm = pin_sensor.get();
      bool valid = mm > 0 && mm < 9999;  // 9999 = sees nothing
      bool fire = false;
      if (valid) {
        last_mm = mm;
        last_valid = now;
        fire = mm <= trigger;
        shown_mm = mm;
      } else {
        // pin got too close for the sensor to read
        fire = last_mm > 0 && last_mm <= trigger + 20 && now - last_valid < 150;
        shown_mm = -1;
      }
      shown_speed = v;
      shown_close = trigger;

      if (armed && fire) {
        shown_closed_mm = valid ? mm : last_mm;
        shown_closed_speed = v;
        close();
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");  // buzz when it grabs something
      }
      pros::delay(LOOP_MS);
    }
  });
}

double pin_mm() { return shown_mm; }
double pin_speed() { return shown_speed; }
double close_mm() { return shown_close; }
double closed_at_mm() { return shown_closed_mm; }
double closed_speed() { return shown_closed_speed; }

void toggle() {
  is_closed ? open() : close();
}

}  // namespace claw
