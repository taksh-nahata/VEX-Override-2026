#pragma once

// Background CSV logger for bench-testing/tuning — the debug screen and
// controller rumble are only good for whatever you happen to catch live;
// this writes every subsystem's numbers to the SD card on its own schedule,
// so a glitch/skip/overshoot that happens too fast to read off the brain
// can be pulled up afterward instead of guessed at. Not tied to opcontrol()
// or autonomous() (runs as its own background task), so it also captures
// PID test moves run via the tuner's B button, which block opcontrol()'s
// own loop while they run.
//
// Read the result with the SD card in any text editor/spreadsheet after a
// test: /usd/log.csv, one row per ~50ms.
namespace sdlog {

// Starts the background logging task. Call once from initialize(). No-op
// (prints a note to the brain instead) if no SD card is inserted.
void start();

}  // namespace sdlog
