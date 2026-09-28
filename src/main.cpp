#include "main.h"

// ----------------------------------------------------------------------------
// GLOBAL DEFINITIONS
// ----------------------------------------------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ----------------------------------------------------------------------------
// HARDWARE CONFIGURATION
// Ports and spin directions live in globals.hpp, not here — we wanted one
// place to check/flip a wire instead of hunting through every subsystem.
// ----------------------------------------------------------------------------

ez::Drive chassis(
    {PORT_DRIVE_LF, PORT_DRIVE_LB},  // Left Ports
    {PORT_DRIVE_RF, PORT_DRIVE_RB},  // Right Ports
    PORT_IMU,
    2.75,        // wheel diameter, inches -- confirmed against the part, not just measured by feel
    600,         // cartridge RPM (blue/6:1) -- TODO(verify): still unconfirmed
    48.0 / 36.0  // external gear ratio (wheel gear / motor gear) -- confirmed:
                 // 36T on the motor, 48T on the wheel
);

ez::tracking_wheel horizontal_tracker(PORT_ODOM_HORIZONTAL, ODOM_HORIZONTAL_WHEEL_DIAMETER, ODOM_HORIZONTAL_OFFSET);

// ----------------------------------------------------------------------------
// AUTON SELECTION -- plain controller buttons, no screen library involved
// at all. We tried a custom LVGL selector (a real header/library version
// mismatch, then a screen that stopped responding after running once),
// then EZ-Template's own LLEMU-based selector (a hard data abort crash)
// -- this avoids every screen-drawing API entirely.
//
// LEFT/RIGHT cycle through the list -- not X/B/A/Y, and not only inside
// disabled(). Without a competition switch, disabled() may never get
// real runtime at all (the robot likely goes straight to driver control),
// so relying on it alone means the selector might never actually be
// interactable. cycle_auton_selection() runs from both disabled() (the
// correct place for a real match, where it DOES get real runtime) and
// opcontrol() (so bench testing without a switch still works), and
// both read/write the exact same selection.
// ----------------------------------------------------------------------------
using AutonFn = void (*)();
struct AutonOption {
  const char* name;
  AutonFn fn;
};
AutonOption auton_options[] = {
    {"Cup+Goal", auton_button_1},
    {"Loader x2", auton_button_2},
    {"Skills", auton_skills},
    {"Drive Test", tune_test},
};
constexpr int AUTON_COUNT = sizeof(auton_options) / sizeof(auton_options[0]);
int auton_index = 2;  // defaults to Skills

void cycle_auton_selection(int line) {
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    auton_index = (auton_index - 1 + AUTON_COUNT) % AUTON_COUNT;
    master.print(0, line, "Auton: %-11s", auton_options[auton_index].name);
  }
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    auton_index = (auton_index + 1) % AUTON_COUNT;
    master.print(0, line, "Auton: %-11s", auton_options[auton_index].name);
  }
}

// ----------------------------------------------------------------------------
// INITIALIZATION
// ----------------------------------------------------------------------------
void initialize() {
  chassis.opcontrol_curve_default_set(2.1, 4.3);
  chassis.odom_tracker_back_set(&horizontal_tracker);  // mounted toward the rear of the robot

  default_constants();
  chassis.initialize();

  lift::initialize();
  // toggle::initialize() -- toggle spinner + color sensor pulled off the
  // robot for now (team's call, hardware in flux). toggle.cpp/hpp still
  // exist, just unused, so this is a one-line add-back whenever it's on
  // the robot again.
  sdlog::start();  // background SD card logging, see sdlog.hpp

  master.print(0, 0, "Auton: %-11s", auton_options[auton_index].name);
}

void disabled() {
  while (true) {
    cycle_auton_selection(0);
    pros::delay(50);
  }
}

void competition_initialize() {}

// ----------------------------------------------------------------------------
// AUTONOMOUS RUNNER
// ----------------------------------------------------------------------------
void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  auton_options[auton_index].fn();
}

// ----------------------------------------------------------------------------
// ANTI-TIP
// We're building a physical anti-tip bar because we don't fully trust
// software here -- once the robot's actually past its balance point,
// cutting motor power usually can't pull it back. Everything below is a
// backstop for the borderline cases, not a replacement for the bar.
//
// Two layers: a speed cap that gets stricter the higher the lift is
// (raising the DR4B raises our center of mass, so what's safe at floor
// height isn't necessarily safe at full height), and a hard cutoff if the
// IMU says we're already tipping.
//
// We added rate-of-tip detection on top of the plain angle check after
// thinking through a case a fixed angle threshold handles badly: a slow
// lean (driving up a bump) shouldn't cut power, but a hard, fast tip
// should cut power well before it reaches the same angle a slow lean
// would eventually hit. Angle alone can't tell those apart early; angle
// AND how fast it's moving can. We also added hysteresis (it has to drop
// back under TIP_RECOVER_DEG, not just under TIP_ANGLE_DEG, to release)
// so the cap doesn't flicker on and off if pitch is bouncing right at the
// threshold from field vibration.
//
// TODO(tune): MAX_LIFT_HEIGHT_DEG, MIN_SPEED_AT_FULL_HEIGHT, TIP_RATE_DEG_S
// — none of these are measured yet.
// TODO(verify): TIP_ANGLE_DEG/TIP_RECOVER_DEG — tip the robot on purpose
// (safely: lift low, bar as backstop) and read pitch/roll off the debug
// screen right before it actually goes over.
// ----------------------------------------------------------------------------
void anti_tip_apply() {
  constexpr double MAX_LIFT_HEIGHT_DEG = 2000;
  constexpr int FULL_SPEED = 127;
  constexpr int MIN_SPEED_AT_FULL_HEIGHT = 70;
  constexpr double TIP_ANGLE_DEG = 15.0;
  constexpr double TIP_RECOVER_DEG = 10.0;   // has to fall back under this, not just under TIP_ANGLE_DEG
  constexpr double TIP_RATE_DEG_S = 60.0;    // how fast counts as "falling," not just "leaning"

  double t = std::clamp(lift::position() / MAX_LIFT_HEIGHT_DEG, 0.0, 1.0);
  int cap = FULL_SPEED - static_cast<int>(t * (FULL_SPEED - MIN_SPEED_AT_FULL_HEIGHT));

  double pitch = chassis.imu.get_pitch();
  double roll = chassis.imu.get_roll();

  static double last_pitch = pitch;
  static double last_roll = roll;
  static std::uint32_t last_ms = pros::millis();
  std::uint32_t now = pros::millis();
  double dt_s = (now - last_ms) / 1000.0;
  double pitch_rate = dt_s > 0 ? (pitch - last_pitch) / dt_s : 0;
  double roll_rate = dt_s > 0 ? (roll - last_roll) / dt_s : 0;
  last_pitch = pitch;
  last_roll = roll;
  last_ms = now;

  double angle = std::max(std::fabs(pitch), std::fabs(roll));
  double rate = std::max(std::fabs(pitch_rate), std::fabs(roll_rate));

  static bool tipping = false;
  if (angle > TIP_ANGLE_DEG || rate > TIP_RATE_DEG_S) {
    tipping = true;
  } else if (angle < TIP_RECOVER_DEG) {
    tipping = false;
  }
  if (tipping) cap = 0;

  // Only calls the setter when the cap actually changes -- EZ-Template
  // re-enables slew every time max speed is set, and we don't want that
  // fighting the driver every single tick.
  static int last_cap = FULL_SPEED;
  if (cap != last_cap) {
    chassis.opcontrol_speed_max_set(cap);
    last_cap = cap;
  }
}

// EXPERIMENTAL -- actively drives the wheels back under the robot's center
// of mass while it's just starting to tip, the same idea a self-balancing
// robot uses. Only fires below anti_tip_apply()'s full cutoff angle, since
// it needs the wheels to still be touching the ground to do anything at
// all -- once we're actually airborne on one end there's no traction left
// to push against.
//
// This only reacts to pitch (forward/back), not roll (side to side) --
// skid-steer can't strafe, so there's genuinely nothing useful the wheels
// can do about a sideways tip. That's a real limit of this drivetrain, not
// something we forgot.
//
// TODO(verify): CORRECTION_SIGN -- prop the robot so the wheels spin
// freely, tilt it by hand, and confirm they spin the direction you'd
// actually want. Get this backwards and it drives further into the tip
// instead of catching it. CORRECTION_SPEED is kept modest on purpose so a
// wrong sign can't do much damage while we're still verifying it.
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
// DEBUG SCREEN (brain) -- for bench testing only, the driver can't see this
// mid-match.
//   Line 0 -- odometry (push the robot right, the number should go up)
//   Line 1 -- lift current/position, TOUCHED/CEILING status
//   Line 2 -- IMU pitch/roll (for the anti-tip sign check above)
//   Line 3 -- claw distance sensor (in) -- for tuning against a ruler
// (toggle target/sees line removed -- toggle spinner + color sensor are
// off the robot for now)
// ----------------------------------------------------------------------------
void debug_screen() {
  pros::screen::print(TEXT_MEDIUM, 0, "odom (in): %.2f", horizontal_tracker.get());

  const char* lift_status = lift::touched_down() ? "TOUCHED" : (lift::at_ceiling_now() ? "CEILING" : "");
  pros::screen::print(TEXT_MEDIUM, 1, "lift mA: %d  pos: %.1f  %s", lift::current_ma(), lift::position(), lift_status);

  pros::screen::print(TEXT_MEDIUM, 2, "pitch/roll: %.1f / %.1f", chassis.imu.get_pitch(), chassis.imu.get_roll());
  pros::screen::print(TEXT_MEDIUM, 3, "claw distance: %.2f in", lift::claw_distance_in());
}

// ----------------------------------------------------------------------------
// CONTROLLER FEEDBACK -- what actually reaches the driver mid-match, since
// they can't see the brain screen. Rumble only fires once per event (on
// the edge), not the whole time it's true, or TOUCHED/CEILING would buzz
// nonstop while pressed against whatever tripped them. Controller text
// only reprints when the value changes -- the link to the controller
// screen is slow, and we found spamming it every tick lags the whole
// thing, not just the display.
// ----------------------------------------------------------------------------
void controller_feedback() {
  static bool was_touched = false;
  static bool was_ceiling = false;

  bool touched = lift::touched_down();
  if (touched && !was_touched) master.rumble(".");
  was_touched = touched;

  bool at_ceiling = lift::at_ceiling_now();
  if (at_ceiling && !was_ceiling) master.rumble("..");
  was_ceiling = at_ceiling;
}

// ----------------------------------------------------------------------------
// MATCH CLOCK
// Override's driver period is a fixed 1:45 (105s). The competition switch
// doesn't tell user code how much time is left, so we just start our own
// timer when opcontrol() begins. That makes this accurate during a real
// match and meaningless during a bench test -- it'll still fire the
// endgame warning 85 seconds after the robot was enabled, whether or not
// that means anything.
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
    master.print(0, 1, "ENDGAME: MIDFIELD");  // line 1 -- line 0 is the auton selection
  }
}

// ----------------------------------------------------------------------------
// DRIVER CONTROL
// ----------------------------------------------------------------------------
void opcontrol() {
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
  match_clock_reset();

  while (true) {
    debug_screen();
    controller_feedback();
    match_clock_update();
    anti_tip_apply();
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    anti_tip_corrective_drive();  // overrides the drive command above if we're actively tipping

    // LEFT/RIGHT also cycle the auton selection here, not just in
    // disabled() -- see the comment above cycle_auton_selection().
    cycle_auton_selection(0);

    // BENCH TEST ONLY -- runs whatever's selected right now, without
    // needing a competition switch. Never in a real match.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      master.print(0, 2, "RUNNING AUTON");
      auton_options[auton_index].fn();
    }

    // Claw
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) claw::toggle();

    // Lift height presets -- one button per height, so the driver doesn't
    // have to eyeball a height with R1/R2 every cycle. Each prints which
    // one got pressed so it's obvious even before the lift finishes
    // moving. R1/R2 immediately take back manual control if pressed
    // during a preset move (see lift::update()).
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      lift::go_to_alliance_goal();
      master.print(0, 2, "ALLIANCE GOAL");
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      lift::go_to_neutral_goal();
      master.print(0, 2, "NEUTRAL GOAL");
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      lift::go_to_center_goal();
      master.print(0, 2, "CENTER GOAL");
    }

    // Lift: R1 raises, R2 lowers, that's the whole manual interface.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lift::update(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lift::update(-127);
    } else {
      lift::update(0);
    }

    // L2 -- toggle spinner is off the robot for now, so this is free.

    pros::delay(ez::util::DELAY_TIME);
  }
}
