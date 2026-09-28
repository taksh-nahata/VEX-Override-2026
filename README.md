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

- `src/main.cpp` — chassis setup, driver control, debug screen, anti-tip, match clock.
- `src/subsystems/` — lift, claw, toggle spinner (one file each).
- `src/autons.cpp` — PID/slew constants, autonomous routines, PID tuner + calibration test moves.
- `src/sdlog.cpp` — background SD card logging (`/usd/log.csv`).
- `include/globals.hpp` — every motor/sensor port and spin direction in one place.
- `TODO.md` — plain-language status/what's-left, no coding background needed to read it.

## Controls

- **R1 / R2** — lift up / down (manual). Pressing either always takes back control from a height preset below.
- **X / B / A** — send the lift to the Alliance / Neutral / Center Goal height preset (each Goal type is a different height). Prints which one you picked to the controller screen.
- **L1** — toggle claw open/closed.
- **L2** (hold) — spin the toggle wheel toward the current target color; stops automatically on reaching it.
- **UP** — swap the toggle target between red/blue. **Y** — set it to yellow. Shown on the controller screen.

`calibrate_straight()`/`calibrate_spin()` (drivetrain/odometry calibration, `autons.cpp`) aren't wired to a button right now. Ask if you want them back on the controller for more drivetrain tuning.

## Picking an auton

Plain controller buttons, no brain screen involved at all — we tried a custom LVGL logo/button screen (real header/library bugs, then a screen that stopped responding after running once), then EZ-Template's own built-in LLEMU selector (which crashed the brain with a data abort), so this avoids every screen API entirely.

**While the robot is disabled** (before a match starts, or any time it isn't in autonomous/driver control): **X** = Cup+Goal, **B** = Loader x2, **A** = Skills, **Y** = Drive Test. Whatever's picked prints to the controller screen (`Auton: ...`), which is what the driver's actually looking at anyway.

For bench testing without a competition switch: once enabled, hold **DOWN** during driver control to run whatever's currently selected, right now. Never used in an actual match.

## Hardware status

Ports are confirmed and in `globals.hpp`. Still open:
- All PID gains (`autons.cpp`, `lift.cpp`) and the toggle color sensor's hue thresholds (`toggle.cpp`) — need tuning against the real robot/toggles.
- The three lift height presets (`PIN_1/2/3_HEIGHT_DEG` in `lift.cpp`) are placeholder guesses — press each button and adjust against the real stack heights.
- Anti-tip constants in `main.cpp` (tip angle/rate, max lift height, correction direction) — unverified, test carefully before relying on them.
- The lift must be powered on with it all the way down at true floor every time — `position()` zeros to wherever it physically is at boot, not a fixed reference.

See `TODO.md` for the full current status and what's next.
