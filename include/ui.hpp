#pragma once

// Team-branded splash + auton selector, built on LVGL. See ui.cpp's header
// comment and logo_image.h for why this calls a handful of LVGL functions
// by their real (v8.3.4) names via manual declarations instead of this
// project's own (mismatched, v9-labeled) liblvgl headers.
namespace ui {

// Shows the logo splash, then builds the selector screen. Call once from
// initialize(), after chassis/lift/etc. are set up (button callbacks
// reference the auton functions immediately, but don't call them until
// autonomous() actually starts).
void init();

// Runs whichever auton is currently selected on the selector screen.
// Defaults to auton_skills if nothing's been tapped yet.
void run_selected();

// Hands the physical screen back to raw pros::screen:: drawing (e.g. the
// opcontrol debug HUD in main.cpp) by clearing the LVGL selector objects.
// run_selected() already does this before running an auton, but on the
// bench — no competition switch connected — the brain goes straight from
// initialize() to opcontrol(), skipping autonomous() (and run_selected())
// entirely. Call this at the top of opcontrol() too so bench testing isn't
// stuck fighting a leftover selector screen. Safe to call twice.
void clear_screen();

}  // namespace ui
