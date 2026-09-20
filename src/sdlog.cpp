#include "main.h"
#include <cstdio>

namespace sdlog {

constexpr std::uint32_t LOG_PERIOD_MS = 50;  // 20 rows/sec -- plenty for a bench test, keeps the file small

void log_loop() {
  FILE* f = fopen("/usd/log.csv", "w");
  if (f == nullptr) {
    pros::screen::print(TEXT_MEDIUM, 6, "SD: couldn't open log.csv");
    return;
  }
  fprintf(f,
          "t_ms,drive_l_in,drive_r_in,heading,pitch,roll,"
          "lift_l_pos,lift_r_pos,lift_l_ma,lift_r_ma,lift_status\n");
  fflush(f);

  std::uint32_t start_ms = pros::millis();
  while (true) {
    const char* lift_status = lift::touched_down() ? "TOUCHED" : (lift::at_ceiling_now() ? "CEILING" : "");
    fprintf(f, "%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%s\n", pros::millis() - start_ms,
            chassis.drive_sensor_left(), chassis.drive_sensor_right(), chassis.imu.get_heading(),
            chassis.imu.get_pitch(), chassis.imu.get_roll(), lift::left_position(), lift::right_position(),
            lift::left_current_ma(), lift::right_current_ma(), lift_status);
    // Flushed every row (not just at close) so a mid-test reset/disable
    // doesn't lose whatever was buffered but not yet written to the card.
    fflush(f);
    pros::delay(LOG_PERIOD_MS);
  }
}

void start() {
  if (!pros::usd::is_installed()) {
    pros::screen::print(TEXT_MEDIUM, 6, "SD: not installed, not logging");
    return;
  }
  pros::Task(log_loop, "sdlog");
}

}  // namespace sdlog
