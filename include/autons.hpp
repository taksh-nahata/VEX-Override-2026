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

// WIP PATH.JERRYIO import -- see autons.cpp for the coordinate transform
// and what's still unconfirmed. Not wired to a button or the selector yet.
void auton_jerryio_test();

// One clean, isolated turn from a reset zero -- no path, no odom math,
// nothing else going on. Turns to a target of 90. Bound to RIGHT in
// main.cpp. Built to answer one question for certain instead of reasoning
// about it from docs/source a third time: does this robot, on this
// EZ-Template version, turn left or right for a positive target? Report
// back which way it actually goes.
void turn_direction_test();
