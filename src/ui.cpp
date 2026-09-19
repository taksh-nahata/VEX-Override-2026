// Doesn't need EZ-Template/chassis, just LVGL directly and the auton
// function declarations — skip main.h's heavy include chain.
#include "api.h"
#include "autons.hpp"
#include "liblvgl/lvgl.h"
#include "logo_image.h"
#include "ui.hpp"

// PROS kernel 4 bundles LVGL 8.3.4 as the actually-compiled
// firmware/liblvgl.a (confirmed via `nm` on the library — it exports
// lv_img_create/lv_btn_create, not this project's own header-declared
// lv_image_create/lv_button_create — and via PROS's release notes), but
// this project's liblvgl HEADERS describe v9, which renamed these and a
// few others. Every declaration below was checked one at a time against
// both (a) `nm firmware/liblvgl.a` for the real exported symbol name and
// (b) LVGL's tagged v8.3.4 source for the real parameter types — nothing
// here is guessed. Only scalar/pointer-only functions are declared this
// way; anything taking a struct BY VALUE (e.g. lv_color_t, which is a
// different SIZE between this project's v9 header — 3 bytes — and the
// real v8.3.4 struct at this build's 32-bit color depth — 4 bytes) is
// avoided entirely, since a by-value struct-size mismatch is a genuine ABI
// hazard, not just a cosmetic one. That's why nothing here sets custom
// colors — only positions, sizes, text, and the logo image.
extern "C" {
struct lv_disp_t;
lv_disp_t* lv_disp_get_default(void);
lv_obj_t* lv_disp_get_scr_act(lv_disp_t* disp);
lv_obj_t* lv_img_create(lv_obj_t* parent);
void lv_img_set_src(lv_obj_t* obj, const void* src);
lv_obj_t* lv_btn_create(lv_obj_t* parent);
}

namespace ui {

lv_obj_t* screen_active() {
  return lv_disp_get_scr_act(lv_disp_get_default());
}

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

void on_option_clicked(lv_event_t* e) {
  auto* opt = static_cast<AutonOption*>(lv_event_get_user_data(e));
  selected_fn = opt->fn;
  lv_label_set_text_fmt(status_label, "Selected: %s", opt->label);
}

void show_splash() {
  lv_obj_t* scr = screen_active();

  lv_obj_t* logo = lv_img_create(scr);
  lv_img_set_src(logo, &team_logo);
  lv_obj_align(logo, LV_ALIGN_CENTER, 0, 0);

  pros::delay(1800);
  lv_obj_clean(scr);
}

void build_selector() {
  lv_obj_t* scr = screen_active();

  lv_obj_t* logo = lv_img_create(scr);
  lv_img_set_src(logo, &team_logo);
  lv_obj_set_size(logo, 60, 60);
  lv_obj_align(logo, LV_ALIGN_TOP_LEFT, 10, 10);

  lv_obj_t* title = lv_label_create(scr);
  lv_label_set_text(title, "Override 2026 - choose auton");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 30, 20);

  constexpr int BTN_W = 200, BTN_H = 60, GAP = 20;
  for (int i = 0; i < OPTION_COUNT; i++) {
    lv_obj_t* btn = lv_btn_create(scr);
    lv_obj_set_size(btn, BTN_W, BTN_H);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 80 + i * (BTN_H + GAP));
    lv_obj_add_event_cb(btn, on_option_clicked, LV_EVENT_CLICKED, &options[i]);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, options[i].label);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  }

  status_label = lv_label_create(scr);
  lv_label_set_text(status_label, "Selected: Skills");
  lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -10);
}

void init() {
  show_splash();
  build_selector();
}

void clear_screen() {
  lv_obj_clean(screen_active());
}

void run_selected() {
  clear_screen();
  selected_fn();
}

}  // namespace ui
