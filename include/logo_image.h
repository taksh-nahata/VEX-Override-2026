#pragma once

#include <cstdint>

// Team logo ("blue."), generated from the PNG the team provided.
//
// PROS kernel 4 bundles LVGL 8.3.4 as the actually-compiled
// firmware/liblvgl.a (confirmed via `nm` on the library, and independently
// via PROS's own release notes) — but this project's liblvgl HEADERS
// describe LVGL v9, whose lv_image_dsc_t has a different bit-packed
// layout. Using the project's own (wrong-version) struct would risk
// misreading memory. LvImgHeaderV8/LvImgDscV8 below are copied
// field-for-field from LVGL's real, tagged v8.3.4 source
// (src/draw/lv_img_buf.h: lv_img_header_t/lv_img_dsc_t) instead of
// guessed — bitfield packing is a compiler+field-order property, not
// tied to which header/name declares it, so an identical local
// declaration compiled by the same toolchain lays out identically to
// what the real library expects. Regenerate the same way if the logo
// ever changes.
struct LvImgHeaderV8 {
  std::uint32_t cf : 5;           // color format — see LV_IMG_CF_TRUE_COLOR_V8 below
  std::uint32_t always_zero : 3;  // must be 0
  std::uint32_t reserved : 2;
  std::uint32_t w : 11;
  std::uint32_t h : 11;
};

struct LvImgDscV8 {
  LvImgHeaderV8 header;
  std::uint32_t data_size;
  const std::uint8_t* data;
};

// LVGL v8.3.4's LV_IMG_CF_TRUE_COLOR enum value — "color format and depth
// should match LV_COLOR settings" (LV_COLOR_DEPTH is 32 in this build's
// lv_conf.h), stored as {blue, green, red, alpha} bytes per pixel.
constexpr std::uint32_t LV_IMG_CF_TRUE_COLOR_V8 = 4;

extern const LvImgDscV8 team_logo;
