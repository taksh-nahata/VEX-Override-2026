#pragma once

void default_constants();

void auton_skills();
void auton_button_1();
void auton_button_2();

// Drive-forward + turn-in-place test move, for exercising a PID live while
// the tuner (see main.cpp's X/B bindings) is adjusting its constants. Not a
// real auton — never wired to the selector.
void tune_test();

// Drivetrain/odometry calibration test moves — see autons.cpp for the full
// explanation of what each one measures and why. Bound to A/LEFT in
// main.cpp. Not real autons either — never wired to the selector.
void calibrate_straight();
void calibrate_spin();
