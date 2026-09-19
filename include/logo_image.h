#pragma once

#include <cstdint>

// Team logo ("blue."), generated from the PNG the team provided by a
// one-off script — a plain raw pixel buffer (0x00RRGGBB per pixel,
// matching pros::Color's own format), blitted with pros::screen::copy_area.
// Deliberately NOT an LVGL image (lv_img_dsc_t/lv_image_dsc_t): this
// project's LVGL headers describe v9's API, but the actual compiled
// firmware/liblvgl.a is an older v8-style build (confirmed by checking its
// real exported symbols — lv_img_create/lv_btn_create, not
// lv_image_create/lv_button_create, and no lv_color_hex/lv_obj_center/
// lv_screen_active at all). The two struct layouts for image data are
// genuinely different, so feeding v9-shaped data to whatever the real
// v8 lv_img_set_src expects would risk silent corruption, not just a
// wrong picture. copy_area sidesteps LVGL entirely. Regenerate the same
// way if the logo ever changes.
extern const std::int32_t LOGO_WIDTH;
extern const std::int32_t LOGO_HEIGHT;
extern const std::uint32_t logo_pixels[];
