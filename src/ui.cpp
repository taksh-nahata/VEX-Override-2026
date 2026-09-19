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

// TODO(missing): auton_button_1/2 and auton_skills are all still empty
// stubs (autons.cpp) — the selector works, there's just nothing to select
// yet.
AutonOption options[] = {
    {"Button 1", auton_button_1},
    {"Button 2", auton_button_2},
    {"Skills", auton_skills},
};
constexpr int OPTION_COUNT = sizeof(options) / sizeof(options[0]);

AutonFn selected_fn = auton_skills;
lv_obj_t* status_label = nullptr;
lv_obj_t* option_buttons[OPTION_COUNT] = {nullptr};

constexpr uint32_t SELECTED_COLOR = 0x2266ff;

void on_option_clicked(lv_event_t* e) {
  auto* opt = static_cast<AutonOption*>(lv_event_get_user_data(e));
  selected_fn = opt->fn;
  lv_label_set_text_fmt(status_label, "Selected: %s", opt->label);

  for (int i = 0; i < OPTION_COUNT; i++) {
    if (options[i].fn == opt->fn) {
      lv_obj_set_style_bg_color(option_buttons[i], lv_color_hex(SELECTED_COLOR), 0);
    } else {
      lv_obj_remove_style(option_buttons[i], nullptr, 0);
    }
  }
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

  constexpr int BTN_W = 200, BTN_H = 60, GAP = 20;
  for (int i = 0; i < OPTION_COUNT; i++) {
    lv_obj_t* btn = lv_btn_create(scr);
    lv_obj_set_size(btn, BTN_W, BTN_H);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 80 + i * (BTN_H + GAP));
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
