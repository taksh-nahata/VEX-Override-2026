#pragma once

// Team logo ("blue."), generated from the PNG the team provided.
//
// Uses the real lv_img_dsc_t from liblvgl/draw/lv_img_buf.h directly — as
// of 2026-09-19 this project's liblvgl headers are the genuine LVGL 8.3.4
// set matching firmware/liblvgl.a (previously they were a mismatched
// newer v9 header set, from copying the compiled firmware over from the
// 2025 project without also copying its matching headers; fixed by
// copying 2025's headers too, since the two were built together).
#include "liblvgl/lvgl.h"

extern const lv_img_dsc_t team_logo;
