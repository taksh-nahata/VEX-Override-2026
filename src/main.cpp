#include "main.h"

ez::Drive chassis(
    {PORT_DRIVE_LF, PORT_DRIVE_LB},  // left
    {PORT_DRIVE_RF, PORT_DRIVE_RB},  // right
    PORT_IMU,
    2.75,        // wheel diameter, inches
    600,         // cartridge rpm
    48.0 / 36.0  // wheel gear / motor gear
);

ez::tracking_wheel horizontal_tracker(PORT_ODOM_HORIZONTAL, ODOM_HORIZONTAL_WHEEL_DIAMETER, ODOM_HORIZONTAL_OFFSET);

// auton selector, LEFT/RIGHT on the controller
struct AutonOption {
  const char* name;
  void (*fn)();
};
AutonOption auton_options[] = {
    {"Cup+Goal R", auton_button_1},
    {"Cup+Goal L", auton_button_1_other_side},
    {"Skills", skills_auton},
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
  master.print(0, 0, "%-15s", auton_options[auton_index].name);  // controller line is 15 chars
}

double level_tilt = 0;  // tilt when sitting flat, the imu isn't perfectly level

void initialize() {
  chassis.opcontrol_curve_default_set(2.1, 4.3);
  chassis.odom_tracker_back_set(&horizontal_tracker);
  default_constants();
  chassis.initialize();
  level_tilt = chassis.imu.get_roll();  // robot has to boot flat

  lift::initialize();
  claw::set_pos_fn([] { return (chassis.drive_sensor_left() + chassis.drive_sensor_right()) / 2; });
  claw::initialize();
  claw::close();  // starts closed

  master.print(0, 0, "%-15s", auton_options[auton_index].name);
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

// split arcade with anti tip. forward/back ramps slower the higher the lift is,
// turns get slower, and the wheels drive into a lean to catch it
void drive_with_anti_tip() {
  constexpr double LIFT_TOP_DEG = 102.8;   // lift position at the top
  constexpr double RAMP_MS_AT_TOP = 400;   // 0 to full speed time with the lift at the top
  constexpr double TURN_CUT_AT_TOP = 0.4;  // turns are 40% slower at the top
  constexpr double LEAN_DEADBAND_DEG = 3;  // ignore leans smaller than this
  constexpr double BALANCE_GAIN = 8;       // speed per degree of lean
  constexpr int BALANCE_SIGN = 1;          // flip if it drives the wrong way

  auto stick = [](pros::controller_analog_e_t axis) {
    int v = master.get_analog(axis);
    return std::abs(v) < 5 ? 0 : v;  // joystick deadband
  };
  double fwd = chassis.opcontrol_curve_left(stick(pros::E_CONTROLLER_ANALOG_LEFT_Y));
  double turn = chassis.opcontrol_curve_right(stick(pros::E_CONTROLLER_ANALOG_RIGHT_X));
  double lift_t = std::clamp(lift::position() / LIFT_TOP_DEG, 0.0, 1.0);  // 0 is down, 1 is top

  double ramp_ms = lift_t * RAMP_MS_AT_TOP;
  double step = ramp_ms < ez::util::DELAY_TIME ? 254 : 127.0 * ez::util::DELAY_TIME / ramp_ms;
  static double fwd_now = 0;
  fwd_now += std::clamp(fwd - fwd_now, -step, step);

  turn *= 1 - TURN_CUT_AT_TOP * lift_t;

  // imu is sideways so roll is the forward/back tilt
  double tilt = chassis.imu.get_roll() - level_tilt;
  double lean = std::fabs(tilt) - LEAN_DEADBAND_DEG;
  double push = lean > 0 ? BALANCE_SIGN * (tilt > 0 ? 1 : -1) * lean * BALANCE_GAIN * lift_t : 0;

  pros::screen::print(TEXT_MEDIUM, 4, "anti-tip: lift %.0f%%  push %.0f        ", lift_t * 100, push);
  chassis.drive_set(std::clamp(fwd_now + turn + push, -127.0, 127.0), std::clamp(fwd_now - turn + push, -127.0, 127.0));
}

// brain screen for testing
void debug_screen() {
  pros::screen::print(TEXT_MEDIUM, 0, "odom (in): %.2f        ", horizontal_tracker.get());
  pros::screen::print(TEXT_MEDIUM, 1, "lift mA: %d  pos: %.1f  raw: %.1f        ", lift::current_ma(), lift::position(),
                      lift::raw_angle());
  pros::screen::print(TEXT_MEDIUM, 2, "pitch/roll: %.1f / %.1f        ", chassis.imu.get_pitch(), chassis.imu.get_roll());
  pros::screen::print(TEXT_MEDIUM, 3, "claw distance: %.2f in        ", lift::claw_distance_in());
  pros::screen::print(TEXT_MEDIUM, 5, "pin %.0f mm  v %.0f  closes at %.0f        ", claw::pin_mm(), claw::pin_speed(),
                      claw::close_mm());
  pros::screen::print(TEXT_MEDIUM, 6, "last auto-close: %.0f mm at v %.0f        ", claw::closed_at_mm(),
                      claw::closed_speed());
}

void opcontrol() {
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);

  while (true) {
    cycle_auton_selection();
    debug_screen();
    drive_with_anti_tip();

    // runs the selected auton for testing, only when not plugged into a field or comp switch
    if (!pros::competition::is_connected() && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      master.print(2, 0, "RUNNING AUTON");
      auton_options[auton_index].fn();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) claw::toggle();

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      lift::go_to_cup_and_pin();
      claw::open();
      claw::close_when_pin();
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
