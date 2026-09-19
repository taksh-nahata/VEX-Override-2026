#include "main.h"

// ----------------------------------------------------------------------------
// GLOBAL DEFINITIONS
// ----------------------------------------------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ----------------------------------------------------------------------------
// HARDWARE CONFIGURATION
// Ports/directions live in globals.hpp — update them there once hardware
// is in hand.
// ----------------------------------------------------------------------------

ez::Drive chassis(
    {PORT_DRIVE_LF, PORT_DRIVE_LB},  // Left Ports
    {PORT_DRIVE_RF, PORT_DRIVE_RB},  // Right Ports
    PORT_IMU,
    2.75,        // Wheel Diameter, inches — pretty sure, double check
    600,         // Cartridge RPM (blue/6:1) — pretty sure
    48.0 / 36.0  // External gear ratio (wheel gear / motor gear): assuming 36T
                 // on the motor side and 48T on the wheel side — flip this to
                 // 36.0/48.0 if that's backwards from how it's actually built
);

ez::tracking_wheel horizontal_tracker(PORT_ODOM_HORIZONTAL, ODOM_HORIZONTAL_WHEEL_DIAMETER, ODOM_HORIZONTAL_OFFSET);

// ----------------------------------------------------------------------------
// BOOT SPLASH
// A code-drawn animation so the screen isn't blank while everything else
// spins up. Runs before the auton selector claims the screen. Swap for a
// real logo later (LVGL image, via the PROS image-converter tool) if wanted.
// ----------------------------------------------------------------------------
void splash_screen() {
  pros::screen::set_pen(pros::Color::black);
  pros::screen::fill_rect(0, 0, 480, 240);
  pros::screen::set_pen(pros::Color::cyan);

  pros::screen::print(TEXT_MEDIUM_CENTER, 4, "TEAM BLUE VEX");
  for (int w = 0; w <= 300; w += 15) {
    pros::screen::fill_rect(90, 110, 90 + w, 130);
    pros::delay(20);
  }

  pros::delay(400);
  pros::screen::set_pen(pros::Color::black);
  pros::screen::fill_rect(0, 0, 480, 240);
}

// ----------------------------------------------------------------------------
// INITIALIZATION
// ----------------------------------------------------------------------------
void initialize() {
  splash_screen();

  ez::ez_template_print();
  pros::delay(500);

  chassis.opcontrol_curve_default_set(2.1, 4.3);
  chassis.odom_tracker_back_set(&horizontal_tracker);  // mounted towards the rear

  default_constants();

  ez::as::auton_selector.autons_add({
      {"Button 1\n\nAuton 1", auton_button_1},
      {"Button 2\n\nAuton 2", auton_button_2},
      {"SKILLS\n\nFull Skills Routine", auton_skills},
  });

  chassis.initialize();
  ez::as::initialize();

  lift::initialize();
  toggle::initialize();
}

void disabled() {}
void competition_initialize() {}

// ----------------------------------------------------------------------------
// AUTONOMOUS RUNNER
// ----------------------------------------------------------------------------
void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  ez::as::auton_selector.selected_auton_call();
}

// ----------------------------------------------------------------------------
// ANTI-TIP
// Neither of these is a real fix. Once the robot's weight has actually
// crossed past its tipping edge, cutting motor power often can't undo it —
// physics has already decided by then. This only helps in borderline cases;
// the more reliable fix for a DR4B that tips at full height is usually
// mechanical (wider base, a wheelie bar) or just not slamming the stick
// with the lift up. Every number below is an unverified guess — TUNE THESE
// ON THE REAL ROBOT, not from anything calculated here.
// ----------------------------------------------------------------------------

// Combines both layers into a single cap so there's one source of truth for
// what's actually applied — computing them separately meant the emergency
// cutoff's 0 could get "stuck" forever, since the preventive half only
// re-applies its own cap when ITS number changes, and would never notice
// the cutoff had overridden it.
//
// Preventive: caps top drive speed based on how high the lift is, since a
// raised DR4B raises the center of mass — input that's safe at floor height
// may not be at full height.
//
// Reactive: if the IMU says the robot is already pitching/rolling past a
// threshold, override down to 0 regardless of height. By the time this
// fires, the robot may already be past the point where it helps — this is
// a backstop, not a save.
//
// Every number below is an unverified guess — TUNE THESE ON THE REAL ROBOT.
// Find TIP_ANGLE_DEG by tipping it (safely, starting at low lift height) and
// watching what get_pitch()/get_roll() report right before it goes.
void anti_tip_apply() {
  constexpr double MAX_LIFT_HEIGHT_DEG = 2000;  // TODO: measure real full-height encoder value
  constexpr int FULL_SPEED = 127;
  constexpr int MIN_SPEED_AT_FULL_HEIGHT = 70;  // TODO: tune — how slow is safe at max height
  constexpr double TIP_ANGLE_DEG = 15.0;

  double t = std::clamp(lift::position() / MAX_LIFT_HEIGHT_DEG, 0.0, 1.0);
  int cap = FULL_SPEED - static_cast<int>(t * (FULL_SPEED - MIN_SPEED_AT_FULL_HEIGHT));

  bool tipping = std::fabs(chassis.imu.get_pitch()) > TIP_ANGLE_DEG || std::fabs(chassis.imu.get_roll()) > TIP_ANGLE_DEG;
  if (tipping) cap = 0;

  // Only calls the setter when the cap actually changes, since EZ-Template
  // re-enables slew whenever max speed is set.
  static int last_cap = FULL_SPEED;
  if (cap != last_cap) {
    chassis.opcontrol_speed_max_set(cap);
    last_cap = cap;
  }
}

// EXPERIMENTAL — actively drives the wheels to try to push the robot's base
// back under its center of mass while it's just starting to tip, the same
// idea self-balancing robots use, overriding the driver's own drive input
// for that instant. Only fires below the full anti_tip_apply() cutoff angle
// (an "early warning" zone), because this can only work while the wheels
// doing the correcting still have ground contact — the moment the robot is
// actually airborne on one end, there's no traction left to act on, so this
// is a narrow window, not a guarantee.
//
// CORRECTION_SIGN is UNVERIFIED. Get it backwards and this drives further
// INTO the tip instead of arresting it. Do not trust this blind: first read
// get_pitch() by hand (print it to the screen, tilt the robot slightly
// yourself with the lift low and a spotter holding it, confirm which sign
// corresponds to which physical direction) before ever letting this touch
// the motors unsupervised. CORRECTION_SPEED is kept deliberately modest so
// a wrong sign can't do much damage while you're still verifying it.
void anti_tip_corrective_drive() {
  constexpr double EARLY_WARNING_DEG = 8.0;  // TODO: tune, smaller than anti_tip_apply's 15°
  constexpr int CORRECTION_SPEED = 40;       // deliberately modest, not full power
  constexpr int CORRECTION_SIGN = 1;         // flip to -1 if testing shows it pushes the wrong way

  double pitch = chassis.imu.get_pitch();
  if (std::fabs(pitch) < EARLY_WARNING_DEG) return;

  int correction = (pitch > 0 ? 1 : -1) * CORRECTION_SIGN * CORRECTION_SPEED;
  chassis.drive_set(correction, correction);
}

// ----------------------------------------------------------------------------
// DEBUG SCREEN
// Live numbers for two bench tests: push the robot right and watch line 0 —
// it should count up; if it counts down, flip DIR_ODOM_HORIZONTAL. Lower the
// lift onto a real stack and watch line 1 — note where it jumps when it
// actually lands vs. normal descending, then set CONTACT_CURRENT_MA
// (lift.cpp) comfortably above the normal number but below the landing
// spike. Line 1 also shows TOUCHED when a stop-on-contact just fired.
// ----------------------------------------------------------------------------
void debug_screen() {
  pros::screen::print(TEXT_MEDIUM, 0, "odom (in): %.2f", horizontal_tracker.get());
  pros::screen::print(TEXT_MEDIUM, 1, "lift mA L/R: %d / %d %s", lift::left_current_ma(), lift::right_current_ma(),
                       lift::touched_down() ? "TOUCHED" : "");
  pros::screen::print(TEXT_MEDIUM, 2, "floor limit: %s (DOWN to toggle)", lift::floor_limit_on() ? "ON" : "OFF");
}

// ----------------------------------------------------------------------------
// DRIVER CONTROL
// ----------------------------------------------------------------------------
void opcontrol() {
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);

  while (true) {
    debug_screen();
    anti_tip_apply();
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    anti_tip_corrective_drive();  // overrides the above if actively tipping

    // Claw
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) claw::toggle();

    // Floor limit toggle — DOWN turns the boot-position floor limit on/off,
    // for troubleshooting (e.g. ruling it in/out of the crooked-lift issue)
    // without editing code. Status shown on line 2 of the debug screen.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) lift::toggle_floor_limit();

    // Lift: R1 = up, R2 = down. Nothing else.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lift::update(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lift::update(-127);
    } else {
      lift::update(0);
    }

    // Toggle spinner — moved off R1 (that's the lift now). Spins while held,
    // but also stops early the instant red is detected. NOTE: the color
    // sensor's hue thresholds (toggle.cpp) are still unverified placeholder
    // guesses — this won't reliably stop on real red until those are
    // calibrated against the actual sensor and toggle.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && toggle::detect() != toggle::Color::RED) {
      toggle::spin(127);
    } else {
      toggle::spin(0);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
