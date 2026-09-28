// Doesn't need EZ-Template/chassis, just LVGL directly and the auton
// function declarations — skip main.h's heavy include chain.
#include "api.h"
#include "autons.hpp"
#include "liblvgl/lvgl.h"
#include "logo_image.h"
#include "ui.hpp"

namespace ui {

using AutonFn = void (*)();

struct AutonOption {
  const char* label;
  AutonFn fn;
};

// TODO(missing): auton_skills() (autons.cpp) is still an empty stub --
// skills is its own game mode with different timing, not planned yet.
// "Drive Test" (tune_test(), autons.cpp) is a diagnostic, not a real
// auton -- an isolated 24in drive + 90deg turn, nothing else, for
// checking whether pid_drive_set()/pid_turn_set() work at all before
// blaming a whole chained auton.
AutonOption options[] = {
    {"Cup+Goal", auton_button_1},
    {"Loader x2", auton_button_2},
    {"Skills", auton_skills},
    {"Drive Test", tune_test},
};
constexpr int OPTION_COUNT = sizeof(options) / sizeof(options[0]);

AutonFn selected_fn = auton_skills;
lv_obj_t* status_label = nullptr;
lv_obj_t* option_buttons[OPTION_COUNT] = {nullptr};

constexpr uint32_t SELECTED_COLOR = 0x2266ff;

// Bench-test convenience: tapping a button on screen runs it 3 seconds
// later, no competition switch/field control needed. Gated on the robot
// actually being enabled (not disabled) at the 3-second mark -- picking
// an auton during the normal pre-match disabled period (which every real
// match starts with) must NOT make the robot start driving on its own.
// `generation` lets a later tap cancel an earlier pending run without
// needing to track/kill the task directly.
constexpr std::uint32_t AUTO_RUN_DELAY_MS = 3000;
int generation = 0;

void run_selected();

void run_after_delay(void* param) {
  int my_generation = static_cast<int>(reinterpret_cast<std::intptr_t>(param));
  pros::delay(AUTO_RUN_DELAY_MS);
  if (my_generation == generation && !pros::competition::is_disabled()) {
    run_selected();
  }
}

void on_option_clicked(lv_event_t* e) {
  auto* opt = static_cast<AutonOption*>(lv_event_get_user_data(e));
  selected_fn = opt->fn;
  lv_label_set_text_fmt(status_label, "Selected: %s (runs in 3s if enabled)", opt->label);

  for (int i = 0; i < OPTION_COUNT; i++) {
    if (options[i].fn == opt->fn) {
      lv_obj_set_style_bg_color(option_buttons[i], lv_color_hex(SELECTED_COLOR), 0);
    } else {
      lv_obj_remove_style(option_buttons[i], nullptr, 0);
    }
  }

  int my_generation = ++generation;
  pros::Task(run_after_delay, reinterpret_cast<void*>(static_cast<std::intptr_t>(my_generation)));
}

void show_splash() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  lv_obj_t* logo = lv_img_create(scr);
  lv_img_set_src(logo, &team_logo);
  lv_obj_center(logo);

  pros::delay(1800);
  lv_obj_clean(scr);
}

void build_selector() {
  lv_obj_t* scr = lv_scr_act();

  lv_obj_t* logo = lv_img_create(scr);
  lv_img_set_src(logo, &team_logo);
  lv_obj_set_size(logo, 60, 60);
  lv_obj_align(logo, LV_ALIGN_TOP_LEFT, 10, 10);

  lv_obj_t* title = lv_label_create(scr);
  lv_label_set_text(title, "Override 2026 - choose auton");
  lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 30, 20);

  // Shrunk from 60/20 to fit 4 buttons on the V5's 240px-tall screen
  // without running into the status label at the bottom.
  constexpr int BTN_W = 200, BTN_H = 30, GAP = 6, START_Y = 66;
  for (int i = 0; i < OPTION_COUNT; i++) {
    lv_obj_t* btn = lv_btn_create(scr);
    lv_obj_set_size(btn, BTN_W, BTN_H);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, START_Y + i * (BTN_H + GAP));
    lv_obj_add_event_cb(btn, on_option_clicked, LV_EVENT_CLICKED, &options[i]);
    option_buttons[i] = btn;

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, options[i].label);
    lv_obj_center(label);
  }

  status_label = lv_label_create(scr);
  lv_label_set_text(status_label, "Selected: Skills");
  lv_obj_set_style_text_color(status_label, lv_color_hex(0xffffff), 0);
  lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -10);
}

void init() {
  show_splash();
  build_selector();
}

void clear_screen() {
  lv_obj_clean(lv_scr_act());
}

void run_selected() {
  clear_screen();
  selected_fn();
}

}  // namespace ui
