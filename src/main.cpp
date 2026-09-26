#include "main.h"

// ----------------------------------------------------------------------------
// GLOBAL DEFINITIONS
// ----------------------------------------------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ----------------------------------------------------------------------------
// HARDWARE CONFIGURATION
// Ports/directions live in globals.hpp.
// ----------------------------------------------------------------------------

ez::Drive chassis(
    {PORT_DRIVE_LF, PORT_DRIVE_LB},  // Left Ports
    {PORT_DRIVE_RF, PORT_DRIVE_RB},  // Right Ports
    PORT_IMU,
    2.75,        // Wheel Diameter, inches — confirmed
    600,         // Cartridge RPM (blue/6:1) — TODO(verify): not confirmed
    48.0 / 36.0  // External gear ratio (wheel gear / motor gear) — confirmed:
                 // 36T motor side, 48T wheel side
);

ez::tracking_wheel horizontal_tracker(PORT_ODOM_HORIZONTAL, ODOM_HORIZONTAL_WHEEL_DIAMETER, ODOM_HORIZONTAL_OFFSET);

// ----------------------------------------------------------------------------
// INITIALIZATION
// ui::init() (src/ui.cpp) owns the whole screen: logo splash into a
// button-based auton selector. See ui.cpp for the LVGL version history.
// ----------------------------------------------------------------------------
void initialize() {
  chassis.opcontrol_curve_default_set(2.1, 4.3);
  chassis.odom_tracker_back_set(&horizontal_tracker);  // mounted towards the rear

  default_constants();
  chassis.initialize();
  chassis.pid_tuner_print_brain_set(true);  // see opcontrol()'s X/B bindings

  lift::initialize();
  toggle::initialize();
  sdlog::start();  // /usd/log.csv on the SD card, see sdlog.hpp

  ui::init();
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
  ui::run_selected();
}

// ----------------------------------------------------------------------------
// ANTI-TIP
// Backstop, not a fix — once the robot's actually past its tipping edge,
// motor power often can't undo it. A physical anti-tip bar is the real
// fix; this just helps in borderline cases underneath it.
//
// Preventive: caps drive speed based on lift height (raised = higher
// center of mass). Reactive: cuts to 0 if the IMU says it's already
// tipping. Combined into one applied value so the reactive cutoff can't
// get stuck overridden by the preventive half.
//
// TODO(tune): MAX_LIFT_HEIGHT_DEG, MIN_SPEED_AT_FULL_HEIGHT — unmeasured.
// TODO(verify): TIP_ANGLE_DEG — tip the robot (safely, lift low, bar as
// backstop) and read pitch/roll off the debug screen right before it goes.
// ----------------------------------------------------------------------------
void anti_tip_apply() {
  constexpr double MAX_LIFT_HEIGHT_DEG = 2000;
  constexpr int FULL_SPEED = 127;
  constexpr int MIN_SPEED_AT_FULL_HEIGHT = 70;
  constexpr double TIP_ANGLE_DEG = 15.0;

  double t = std::clamp(lift::position() / MAX_LIFT_HEIGHT_DEG, 0.0, 1.0);
  int cap = FULL_SPEED - static_cast<int>(t * (FULL_SPEED - MIN_SPEED_AT_FULL_HEIGHT));

  bool tipping = std::fabs(chassis.imu.get_pitch()) > TIP_ANGLE_DEG || std::fabs(chassis.imu.get_roll()) > TIP_ANGLE_DEG;
  if (tipping) cap = 0;

  // Only calls the setter when the cap changes -- EZ-Template re-enables
  // slew whenever max speed is set.
  static int last_cap = FULL_SPEED;
  if (cap != last_cap) {
    chassis.opcontrol_speed_max_set(cap);
    last_cap = cap;
  }
}

// EXPERIMENTAL — actively drives the wheels to push the robot's base back
// under its center of mass while it's just starting to tip. Only fires
// below anti_tip_apply()'s full cutoff angle (an early-warning zone) since
// it needs the wheels to still have ground contact to do anything.
//
// TODO(verify): CORRECTION_SIGN — prop the robot so the wheels spin
// freely, tilt by hand, confirm they spin the expected direction. Wrong
// sign drives further into the tip. CORRECTION_SPEED kept modest so a
// wrong sign can't do much damage while verifying.
void anti_tip_corrective_drive() {
  constexpr double EARLY_WARNING_DEG = 8.0;
  constexpr int CORRECTION_SPEED = 40;
  constexpr int CORRECTION_SIGN = 1;

  double pitch = chassis.imu.get_pitch();
  if (std::fabs(pitch) < EARLY_WARNING_DEG) return;

  int correction = (pitch > 0 ? 1 : -1) * CORRECTION_SIGN * CORRECTION_SPEED;
  chassis.drive_set(correction, correction);
}

// ----------------------------------------------------------------------------
// DEBUG SCREEN (brain) — bench-testing only, driver can't see this mid-match.
//   Line 0 — odometry (push the robot right, should count up)
//   Line 1 — lift current/position, TOUCHED/CEILING status
//   Line 2 — IMU pitch/roll (anti-tip sign check)
//   Line 3 — toggle target vs. detected color
// ----------------------------------------------------------------------------
void debug_screen() {
  pros::screen::print(TEXT_MEDIUM, 0, "odom (in): %.2f", horizontal_tracker.get());

  const char* lift_status = lift::touched_down() ? "TOUCHED" : (lift::at_ceiling_now() ? "CEILING" : "");
  pros::screen::print(TEXT_MEDIUM, 1, "lift mA: %d  pos: %.1f  %s", lift::current_ma(), lift::position(), lift_status);

  pros::screen::print(TEXT_MEDIUM, 2, "pitch/roll: %.1f / %.1f", chassis.imu.get_pitch(), chassis.imu.get_roll());
  pros::screen::print(TEXT_MEDIUM, 3, "toggle target: %s  sees: %s", toggle::color_name(toggle::target_color()),
                       toggle::color_name(toggle::detect()));
}

// ----------------------------------------------------------------------------
// CONTROLLER FEEDBACK — the actual driver-facing feedback (rumble + the
// controller's own 3-line screen), since the brain isn't visible mid-match.
// Rumble is edge-triggered (once per event, not held). Controller print
// only fires on change -- the wireless link to it is slow, spamming it
// every tick lags the whole link.
// ----------------------------------------------------------------------------
void controller_feedback() {
  static bool was_touched = false;
  static bool was_ceiling = false;
  static bool was_on_target = false;
  static toggle::Color shown_target = toggle::Color::NONE;  // forces the first print

  bool touched = lift::touched_down();
  if (touched && !was_touched) master.rumble(".");
  was_touched = touched;

  bool at_ceiling = lift::at_ceiling_now();
  if (at_ceiling && !was_ceiling) master.rumble("..");
  was_ceiling = at_ceiling;

  bool on_target = toggle::detect() == toggle::target_color();
  if (on_target && !was_on_target) master.rumble("-");
  was_on_target = on_target;

  // Controller screen line 0: always shows the current toggle target, not
  // just right after UP/Y is pressed -- driver can glance at it anytime.
  toggle::Color current_target = toggle::target_color();
  if (current_target != shown_target) {
    master.print(0, 0, "target: %-6s", toggle::color_name(current_target));
    shown_target = current_target;
  }
}

// ----------------------------------------------------------------------------
// MATCH CLOCK
// Override's driver period is a fixed 1:45 (105s). The competition switch
// doesn't broadcast time remaining to user code, so this starts a timer
// when opcontrol() begins -- accurate for a real match, meaningless
// (fires the warning at 85s in regardless) during an untimed bench test.
// ----------------------------------------------------------------------------
constexpr std::uint32_t MATCH_DURATION_MS = 105000;
constexpr std::uint32_t ENDGAME_WARNING_MS = 20000;  // Override's contested-Midfield window

std::uint32_t opcontrol_start_ms = 0;
bool endgame_warned = false;

void match_clock_reset() {
  opcontrol_start_ms = pros::millis();
  endgame_warned = false;
}

void match_clock_update() {
  std::uint32_t elapsed = pros::millis() - opcontrol_start_ms;
  if (!endgame_warned && elapsed >= MATCH_DURATION_MS - ENDGAME_WARNING_MS) {
    endgame_warned = true;
    master.rumble("- - -");
    master.print(0, 1, "ENDGAME: MIDFIELD");  // line 1 -- line 0 is the toggle target
  }
}

// ----------------------------------------------------------------------------
// DRIVER CONTROL
// ----------------------------------------------------------------------------
void opcontrol() {
  ui::clear_screen();  // no-op if run_selected() already did this
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
  match_clock_reset();

  while (true) {
    // Drivetrain PID tuner (EZ-Template built-in) -- X toggles it, B runs
    // tune_test() (autons.cpp). Once on: Up/Down picks the PID set (Turn
    // = the IMU-based one), Left/Right adjusts the selected value. Skips
    // debug_screen() and the UP/LEFT bindings below so they don't
    // double-fire against the tuner's own Up/Down/Left/Right.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) chassis.pid_tuner_toggle();
    chassis.pid_tuner_iterate();
    if (chassis.pid_tuner_enabled()) {
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) tune_test();
    } else {
      debug_screen();
    }

    controller_feedback();
    match_clock_update();
    anti_tip_apply();
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    anti_tip_corrective_drive();  // overrides the above if actively tipping

    // Claw
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) claw::toggle();

    // Toggle target color -- UP swaps red/blue, Y sets yellow directly.
    // Shown continuously on the controller screen (controller_feedback()).
    if (!chassis.pid_tuner_enabled() && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      toggle::toggle_target_red_blue();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      toggle::set_target_yellow();
    }

    // Drivetrain/odometry calibration test moves (autons.cpp) -- A runs
    // calibrate_straight() (report the tape-measured distance back), LEFT
    // runs calibrate_spin() (fully automatic).
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) calibrate_straight();
    if (!chassis.pid_tuner_enabled() && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      calibrate_spin();
    }

    // Lift: R1 = up, R2 = down. Nothing else.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lift::update(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lift::update(-127);
    } else {
      lift::update(0);
    }

    // Toggle spinner -- spins while held, stops early on reaching the
    // target color. TODO(tune): toggle.cpp's hue thresholds are still
    // unverified placeholder guesses.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && toggle::detect() != toggle::target_color()) {
      toggle::spin(127);
    } else {
      toggle::spin(0);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
