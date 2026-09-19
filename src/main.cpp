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
    2.75,        // Wheel Diameter, inches — TODO(verify): confirmed "pretty sure" 2026-09-13, not measured
    600,         // Cartridge RPM (blue/6:1) — TODO(verify): same as above
    48.0 / 36.0  // External gear ratio (wheel gear / motor gear): TODO(verify) —
                 // assuming 36T on the motor side and 48T on the wheel side;
                 // flip to 36.0/48.0 if that's backwards from how it's built
);

ez::tracking_wheel horizontal_tracker(PORT_ODOM_HORIZONTAL, ODOM_HORIZONTAL_WHEEL_DIAMETER, ODOM_HORIZONTAL_OFFSET);

// ----------------------------------------------------------------------------
// INITIALIZATION
// ui::init() (src/ui.cpp) owns the whole screen: logo splash straight into
// a real button-based selector, replacing both the old hand-drawn splash
// and EZ-Template's own selector. It calls a few LVGL functions by their
// real (v8.3.4) names via manual declarations, since this build's
// firmware/liblvgl.a is compiled as LVGL 8.3.4 while this project's LVGL
// headers describe v9 — see ui.cpp's header comment for the full story and
// what was verified before relying on it. ez::ez_template_print() and
// ez::as::initialize()/autons_add() are gone — both drew to the legacy LCD
// emulator (LLEMU) or EZ-Template's own selector, which ui:: now owns.
// ----------------------------------------------------------------------------
void initialize() {
  chassis.opcontrol_curve_default_set(2.1, 4.3);
  chassis.odom_tracker_back_set(&horizontal_tracker);  // mounted towards the rear

  default_constants();
  chassis.initialize();

  lift::initialize();
  toggle::initialize();

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
// Neither layer here is a real fix. Once the robot's weight has actually
// crossed past its tipping edge, cutting or redirecting motor power often
// can't undo it — physics has already decided by then. This only helps in
// borderline cases. The team is adding a physical anti-tip bar (2026-09-18)
// as the actual reliable fix; these are a backstop underneath that, not a
// replacement for it.
// ----------------------------------------------------------------------------

// Combines a preventive speed cap and a reactive cutoff into one applied
// value, so there's one source of truth for what's actually sent to the
// motors — computing them separately meant the reactive cutoff's 0 could
// get "stuck" forever, since the preventive half only re-applies its own
// cap when ITS number changes, and would never notice the cutoff had
// overridden it.
//
// Preventive: caps top drive speed based on how high the lift is, since a
// raised DR4B raises the center of mass — input that's safe at floor height
// may not be at full height.
//
// Reactive: if the IMU says the robot is already pitching/rolling past
// TIP_ANGLE_DEG, override down to 0 regardless of height. By the time this
// fires, the robot may already be past the point where it helps.
//
// TODO(tune): MAX_LIFT_HEIGHT_DEG — measure the real full-height encoder
// value once the ceiling is known (see lift.cpp's at_ceiling_now()).
// TODO(tune): MIN_SPEED_AT_FULL_HEIGHT — how slow is actually safe at max
// height.
// TODO(verify): TIP_ANGLE_DEG — find by tipping the robot (safely, low lift
// height, with the new anti-tip bar as a backstop) and watching line 3 of
// the debug screen for what pitch/roll actually read right before it goes.
void anti_tip_apply() {
  constexpr double MAX_LIFT_HEIGHT_DEG = 2000;
  constexpr int FULL_SPEED = 127;
  constexpr int MIN_SPEED_AT_FULL_HEIGHT = 70;
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
// for that instant. Only fires below anti_tip_apply()'s full cutoff angle
// (an "early warning" zone), because this can only work while the wheels
// doing the correcting still have ground contact — the moment the robot is
// actually airborne on one end, there's no traction left to act on, so this
// is a narrow window, not a guarantee.
//
// Sign check 2026-09-18: tipping backward reads as negative pitch on this
// robot; negative drive_set() is the standard "backward" convention, so
// CORRECTION_SIGN = 1 is likely already correct by that reasoning — but
// TODO(verify): prop the robot so the wheels spin freely, tilt it by hand,
// and confirm the wheels actually spin the expected direction before
// trusting this unsupervised. Get it backwards and it drives further INTO
// the tip instead of arresting it. CORRECTION_SPEED is kept deliberately
// modest so a wrong sign can't do much damage while still verifying it.
void anti_tip_corrective_drive() {
  constexpr double EARLY_WARNING_DEG = 8.0;  // TODO(tune): smaller than anti_tip_apply's 15°
  constexpr int CORRECTION_SPEED = 40;       // deliberately modest, not full power
  constexpr int CORRECTION_SIGN = 1;         // TODO(verify): see comment above

  double pitch = chassis.imu.get_pitch();
  if (std::fabs(pitch) < EARLY_WARNING_DEG) return;

  int correction = (pitch > 0 ? 1 : -1) * CORRECTION_SIGN * CORRECTION_SPEED;
  chassis.drive_set(correction, correction);
}

// ----------------------------------------------------------------------------
// DEBUG SCREEN
// Live numbers for bench testing:
//   Line 0 — odometry. Push the robot right, should count up.
//   Line 1 — lift motor current, and TOUCHED/CEILING when a contact-stop
//            just fired (see lift.cpp). Use this to tune CONTACT_CURRENT_MA
//            and CEILING_CURRENT_MA there.
//   Line 2 — floor limit status (DOWN toggles it).
//   Line 3 — IMU pitch/roll, for the anti-tip sign check above.
// ----------------------------------------------------------------------------
void debug_screen() {
  pros::screen::print(TEXT_MEDIUM, 0, "odom (in): %.2f", horizontal_tracker.get());

  const char* lift_status = lift::touched_down() ? "TOUCHED" : (lift::at_ceiling_now() ? "CEILING" : "");
  pros::screen::print(TEXT_MEDIUM, 1, "lift mA L/R: %d / %d %s", lift::left_current_ma(), lift::right_current_ma(),
                       lift_status);

  pros::screen::print(TEXT_MEDIUM, 2, "floor limit: %s (DOWN to toggle)", lift::floor_limit_on() ? "ON" : "OFF");
  pros::screen::print(TEXT_MEDIUM, 3, "pitch/roll: %.1f / %.1f", chassis.imu.get_pitch(), chassis.imu.get_roll());
}

// ----------------------------------------------------------------------------
// CONTROLLER FEEDBACK
// The brain's own screen (debug_screen() above) is only useful for bench
// testing — the driver can't see it mid-match, it's mounted on the robot.
// This is the actual driver-facing feedback: the controller's rumble motor
// and its own small 3-line screen. Rumble fires once per event (edge-
// triggered), not every tick the condition holds, or TOUCHED/CEILING would
// buzz continuously for as long as you're pressed against whatever tripped
// them. Controller print is rate-limited separately — it's a slow wireless
// link ("controller text update rate is slow" per the PROS docs); spamming
// it every 20ms tick lags the whole link, not just the display.
// ----------------------------------------------------------------------------
void controller_feedback() {
  static bool was_touched = false;
  static bool was_ceiling = false;
  static bool was_red = false;

  bool touched = lift::touched_down();
  if (touched && !was_touched) master.rumble(".");
  was_touched = touched;

  bool at_ceiling = lift::at_ceiling_now();
  if (at_ceiling && !was_ceiling) master.rumble("..");
  was_ceiling = at_ceiling;

  bool red = toggle::detect() == toggle::Color::RED;
  if (red && !was_red) master.rumble("-");
  was_red = red;
}

// ----------------------------------------------------------------------------
// MATCH CLOCK
// Override's driver period is a fixed 1:45 (105s) per the game manual. The
// V5 competition switch doesn't broadcast time remaining to user code, so
// this just starts a timer the moment opcontrol() begins and counts down —
// accurate for a real timed match, meaningless during an untimed bench
// test (it'll still fire the endgame warning at 85s in regardless of
// whether anyone's actually 20s from the real end of anything).
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
    master.print(0, 0, "ENDGAME: MIDFIELD");
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
    debug_screen();
    controller_feedback();
    match_clock_update();
    anti_tip_apply();
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    anti_tip_corrective_drive();  // overrides the above if actively tipping

    // Claw
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) claw::toggle();

    // Floor limit toggle — DOWN turns it on/off, for troubleshooting the
    // crooked-lift issue without editing code. Status on line 2. (The
    // ceiling has no equivalent toggle — it's automatic, see lift.cpp.)
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) lift::toggle_floor_limit();

    // Lift: R1 = up, R2 = down. Nothing else.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lift::update(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lift::update(-127);
    } else {
      lift::update(0);
    }

    // Toggle spinner — moved off R1 (that's the lift now). Spins while
    // held, but also stops early the instant red is detected.
    // TODO(tune): toggle.cpp's hue thresholds are still unverified
    // placeholder guesses — this won't reliably stop on real red until
    // those are calibrated against the actual sensor and toggle.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && toggle::detect() != toggle::Color::RED) {
      toggle::spin(127);
    } else {
      toggle::spin(0);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
