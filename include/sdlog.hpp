#pragma once

// We built this after guessing wrong at CEILING_CURRENT_MA twice in a row
// -- the debug screen only shows you whatever you happen to catch live,
// and a glitch that lasts 100ms is basically impossible to read off a
// scrolling brain screen in real time. This writes every subsystem's
// numbers to the SD card on its own schedule instead, so we can pull up
// exactly what happened after the fact rather than guess again. It runs
// as its own background task, not tied to opcontrol()/autonomous(), so it
// keeps logging through the tuner's test moves too, which block
// opcontrol()'s own loop while they run.
//
// Read the result with the SD card in any text editor/spreadsheet after a
// test: /usd/log.csv, one row per ~50ms.
namespace sdlog {

// Starts the background logging task. Call once from initialize(). No-op
// (prints a note to the brain instead) if no SD card is inserted.
void start();

}  // namespace sdlog
