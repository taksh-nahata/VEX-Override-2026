#pragma once

namespace claw {

void set_pos_fn(double (*inches)());  // robot position in inches, for the auto close
void initialize();

void close_when_pin();  // closes by itself when a pin gets close, open/close/toggle cancel it

// for the brain screen
double pin_mm();
double pin_speed();
double close_mm();
double closed_at_mm();
double closed_speed();

bool closed();
void open();
void close();
void toggle();

}  // namespace claw
