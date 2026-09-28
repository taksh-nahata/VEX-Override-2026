# teamblueVEX — Override (2026-2027)

Robot code for the VEX V5RC **Override** season, built on [PROS](https://pros.cs.purdue.edu/) + [EZ-Template](https://ez-robotics.github.io/EZ-Template/).

## Setup (from a brand new computer)

1. **Install [VS Code](https://code.visualstudio.com/)**.
2. **Install the PROS extension**: open VS Code's Extensions tab (`Ctrl+Shift+X`), search **"PROS"**, install it ([marketplace link](https://marketplace.visualstudio.com/items?itemName=sigbots.pros)). When it prompts you to install the PROS toolchain, accept — this installs everything needed to build V5 code, no separate download required. Full walkthrough: [PROS Getting Started](https://pros.cs.purdue.edu/v5/getting-started/index.html).
3. **Get this code onto your computer** — either:
   - `git clone https://github.com/<org>/<repo>.git`, or
   - [GitHub Desktop](https://desktop.github.com/) if you'd rather not use the command line: File → Clone Repository → paste this repo's URL.
4. **Open the project**: in VS Code, `File → Open Folder`, and select the cloned folder itself (the one with `project.pros` in it) — not a parent folder. The PROS extension activates automatically once it sees `project.pros`.
5. **Build**: click the PROS icon in the left sidebar → Build (or `Ctrl+Shift+P` → "PROS: Build Project"). First build takes longer; after that it's quick.
6. **Upload to the robot**: plug in the V5 brain via USB, then PROS sidebar → Upload (or "PROS: Upload Project").

No prior PROS/VEX coding experience needed to get this far — steps 1-2 are one-time per computer.

## Project layout

- `src/main.cpp` — chassis setup, driver control, auton selection, debug screen, anti-tip, match clock.
- `src/subsystems/` — lift, claw, toggle spinner (one file each). Toggle spinner + color sensor are off the robot for now (team's call) — the code's still there, just unused.
- `src/autons.cpp` — PID/slew constants, autonomous routines, PID tuner + calibration test moves.
- `src/sdlog.cpp` — background SD card logging (`/usd/log.csv`).
- `include/globals.hpp` — every motor/sensor port and spin direction in one place.
- `TODO.md` — plain-language status/what's-left, no coding background needed to read it.

## Controls

- **R1 / R2** — lift up / down (manual). Pressing either always takes back control from a height preset below.
- **X / B / A** — send the lift to the Alliance / Neutral / Center Goal height preset (each Goal type is a different height). Prints which one you picked to the controller screen.
- **LEFT / RIGHT** — cycle the auton selection (see below).
- **DOWN** (driver control only) — bench-test the current auton selection right now, no competition switch needed. Never used in a real match.
- **L1** — toggle claw open/closed.

UP, Y, and L2 are free right now (used to be toggle spinner controls). `calibrate_straight()`/`calibrate_spin()` (drivetrain/odometry calibration, `autons.cpp`) also aren't wired to a button.

## Picking an auton

Plain controller buttons, no brain screen involved at all — we tried a custom LVGL logo/button screen (real header/library bugs, then a screen that stopped responding after running once), then EZ-Template's own built-in LLEMU selector (which crashed the brain with a data abort), so this avoids every screen-drawing API entirely.

Press **LEFT/RIGHT** to cycle through the options (Cup+Goal, Loader x2, Skills, Drive Test) — works both while the robot is disabled (the normal pre-match state) and during driver control, since without a competition switch the robot may never sit in "disabled" long enough to pick anything there. Whatever's selected prints to the controller screen (`Auton: ...`), which is what the driver's actually looking at anyway. Hold **DOWN** during driver control to run it immediately for bench testing.

## Hardware status

Ports are confirmed and in `globals.hpp`. Still open:
- All PID gains (`autons.cpp`, `lift.cpp`) — need tuning against the real robot.
- The lift height presets (`lift.cpp`) are placeholder guesses — press each button and adjust against the real Goals/stacks.
- Anti-tip constants in `main.cpp` (tip angle/rate, max lift height, correction direction) — unverified, test carefully before relying on them.
- The lift must be powered on with it all the way down at true floor every time — `position()` zeros to wherever it physically is at boot, not a fixed reference.

See `TODO.md` for the full current status and what's next.
