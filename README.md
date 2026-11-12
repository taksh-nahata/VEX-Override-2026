# teamblueVEX Override 2026-27

Code for our V5RC Override robot. PROS 4 + EZ-Template.

## Setup

1. Install VS Code and the PROS extension (it installs the toolchain for you).
2. Clone this repo and open the folder with `project.pros` in it.
3. PROS sidebar: Build, then Upload with the brain plugged in.

## Files

- `src/main.cpp` - chassis, driver control, anti tip, auton selector
- `src/autons.cpp` - pid constants and the autons
- `src/subsystems/lift.cpp` - lift
- `src/subsystems/claw.cpp` - claw and the auto close
- `include/globals.hpp` - all the ports

## Controls

- Left stick / right stick: drive
- R1 / R2: lift up / down
- A: lift to cup + pin height, opens the claw, and the claw closes by itself when a pin gets close
- L1: open / close the claw
- LEFT / RIGHT: pick the auton (shows on the controller)
- DOWN: run the selected auton from driver control, for testing

## Autons

Cup+Goal R, Cup+Goal L, Skills, Drive Test.

## Still to tune

- pid constants in `autons.cpp`
- lift numbers at the top of `lift.cpp`
- auto close numbers at the top of `claw.cpp`
