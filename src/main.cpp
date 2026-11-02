#include "main.h"

// Ports and directions are in globals.hpp.
ez::Drive chassis(
    {PORT_DRIVE_LF, PORT_DRIVE_LB},  // left
    {PORT_DRIVE_RF, PORT_DRIVE_RB},  // right
    PORT_IMU,
    2.75,        // wheel diameter, inches
    600,         // cartridge RPM (blue) -- TODO(verify)
    48.0 / 36.0  // 36T on the motor, 48T on the wheel
);

ez::tracking_wheel horizontal_tracker(PORT_ODOM_HORIZONTAL, ODOM_HORIZONTAL_WHEEL_DIAMETER, ODOM_HORIZONTAL_OFFSET);

// ----------------------------------------------------------------------------
// AUTON SELECTION -- LEFT/RIGHT on the controller, no brain-screen selector
// (both screen selectors we tried crashed). Works from disabled() for real
// matches and from opcontrol() for bench testing without a competition switch.
// ----------------------------------------------------------------------------
struct AutonOption {
  const char* name;
  void (*fn)();
};
AutonOption auton_options[] = {
    {"Cup+Goal R", auton_button_1},
    {"Cup+Goal L", auton_button_1_other_side},
    {"Drive Test", tune_test},
};
constexpr int AUTON_COUNT = sizeof(auton_options) / sizeof(auton_options[0]);
int auton_index = 0;

void cycle_auton_selection() {
  int step = 0;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) step = -1;
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) step = 1;
  if (step == 0) return;
  auton_index = (auton_index + step + AUTON_COUNT) % AUTON_COUNT;
  master.print(0, 0, "%-15s", auton_options[auton_index].name);  // controller line is 15 chars wide
}

double level_tilt = 0;  // forward/back tilt when sitting flat (the IMU isn't mounted perfectly level)

void initialize() {
  chassis.opcontrol_curve_default_set(2.1, 4.3);
  chassis.odom_tracker_back_set(&horizontal_tracker);
  default_constants();
  chassis.initialize();
  level_tilt = chassis.imu.get_roll();  // robot must boot sitting flat

  lift::initialize();
  claw::close();  // start preloaded/closed

  master.print(0, 0, "%-15s", auton_options[auton_index].name);  // controller line is 15 chars wide
}

void disabled() {
  while (true) {
    cycle_auton_selection();
    pros::delay(50);
  }
}

void competition_initialize() {}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  auton_options[auton_index].fn();
}

// ----------------------------------------------------------------------------
// DRIVER DRIVE + ANTI-TIP (split arcade)
// With the lift up, hard starts/stops tip the robot. So forward/back speed
// ramps instead of jumping (instant with the lift down, 400ms at the top).
// Turns aren't ramped (that made it keep spinning), just slower with the lift up.
// A forward/back lean past 3 degrees drives the wheels toward the lean, back
// under the robot -- scaled by lift height, so nothing happens with it down.
// ----------------------------------------------------------------------------
void drive_with_anti_tip() {
  constexpr double LIFT_TOP_DEG = 102.8;   // lift "pos" at the top (measured 2026-10-18)
  constexpr double RAMP_MS_AT_TOP = 400;   // 0 -> full speed time with the lift at the top
  constexpr double TURN_CUT_AT_TOP = 0.4;  // turns are 40% slower with the lift at the top
  constexpr double LEAN_DEADBAND_DEG = 3;  // ignore leans smaller than this
  constexpr double BALANCE_GAIN = 8;       // drive speed per degree of lean past the deadband
  constexpr int BALANCE_SIGN = 1;          // bench-tested 2026-10-18

  auto stick = [](pros::controller_analog_e_t axis) {
    int v = master.get_analog(axis);
    return std::abs(v) < 5 ? 0 : v;  // joystick deadband
  };
  double fwd = chassis.opcontrol_curve_left(stick(pros::E_CONTROLLER_ANALOG_LEFT_Y));
  double turn = chassis.opcontrol_curve_right(stick(pros::E_CONTROLLER_ANALOG_RIGHT_X));
  double lift_t = std::clamp(lift::position() / LIFT_TOP_DEG, 0.0, 1.0);  // 0 = down, 1 = top

  double ramp_ms = lift_t * RAMP_MS_AT_TOP;
  double step = ramp_ms < ez::util::DELAY_TIME ? 254 : 127.0 * ez::util::DELAY_TIME / ramp_ms;
  static double fwd_now = 0;
  fwd_now += std::clamp(fwd - fwd_now, -step, step);

  turn *= 1 - TURN_CUT_AT_TOP * lift_t;

  // The IMU is mounted sideways, so ROLL is our forward/back tilt.
  double tilt = chassis.imu.get_roll() - level_tilt;
  double lean = std::fabs(tilt) - LEAN_DEADBAND_DEG;
  double push = lean > 0 ? BALANCE_SIGN * (tilt > 0 ? 1 : -1) * lean * BALANCE_GAIN * lift_t : 0;

  pros::screen::print(TEXT_MEDIUM, 4, "anti-tip: lift %.0f%%  push %.0f        ", lift_t * 100, push);  // bench check
  chassis.drive_set(std::clamp(fwd_now + turn + push, -127.0, 127.0), std::clamp(fwd_now - turn + push, -127.0, 127.0));
}

// Brain screen, for bench testing (the driver can't see it mid-match).
void debug_screen() {
  pros::screen::print(TEXT_MEDIUM, 0, "odom (in): %.2f        ", horizontal_tracker.get());
  pros::screen::print(TEXT_MEDIUM, 1, "lift mA: %d  pos: %.1f  raw: %.1f        ", lift::current_ma(), lift::position(),
                      lift::raw_angle());
  pros::screen::print(TEXT_MEDIUM, 2, "pitch/roll: %.1f / %.1f        ", chassis.imu.get_pitch(), chassis.imu.get_roll());
  pros::screen::print(TEXT_MEDIUM, 3, "claw distance: %.2f in        ", lift::claw_distance_in());
}

void opcontrol() {
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);

  while (true) {
    cycle_auton_selection();
    debug_screen();
    drive_with_anti_tip();

    // BENCH TEST ONLY -- runs the selected auton without a competition switch.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      master.print(2, 0, "RUNNING AUTON");
      auton_options[auton_index].fn();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) claw::toggle();

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {  // R1/R2 cancel it
      lift::go_to_cup_and_pin();
      claw::open();
      master.print(2, 0, "CUP + PIN      ");
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      lift::update(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lift::update(-127);
    } else {
      lift::update(0);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
