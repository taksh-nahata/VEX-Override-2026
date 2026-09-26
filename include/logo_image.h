#pragma once

// Team logo ("blue."), generated from the PNG the team gave us.
//
// Built on the real lv_img_dsc_t from liblvgl/draw/lv_img_buf.h. Worth
// knowing if you're touching this: our liblvgl headers have to be the
// genuine LVGL 8.3.4 set that actually matches firmware/liblvgl.a. We got
// bitten once by a mismatched v9 header set that crept in from copying
// the compiled firmware over without its matching headers -- fixed by
// copying both from the same source together.
#include "liblvgl/lvgl.h"

extern const lv_img_dsc_t team_logo;
