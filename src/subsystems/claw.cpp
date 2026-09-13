// Doesn't touch EZ-Template/chassis, so skip main.h's heavy include chain
// (drive.hpp, <bits/stdc++.h> via EZ-Template/util.hpp) to keep this file's
// compile time down.
#include "api.h"
#include "globals.hpp"
#include "subsystems/claw.hpp"

namespace claw {

pros::adi::DigitalOut piston(PORT_CLAW_SOLENOID);
bool is_closed = false;

void open() {
  piston.set_value(false);
  is_closed = false;
}

void close() {
  piston.set_value(true);
  is_closed = true;
}

void toggle() {
  is_closed ? open() : close();
}

}  // namespace claw
