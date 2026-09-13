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

- `src/main.cpp` — chassis setup, driver control, initialization.
- `src/subsystems/` — lift, claw, toggle spinner (one file each).
- `src/autons.cpp` — PID tuning + autonomous routines.
- `include/globals.hpp` — every motor/sensor port and spin direction in one place.

## Hardware status

Ports are confirmed and in `globals.hpp`. Still open:
- `DIR_ODOM_HORIZONTAL` — needs a push-the-robot-right bench check.
- Wheel diameter / gear ratio in `main.cpp` — marked "pretty sure," worth double-checking.
- All PID gains (`autons.cpp`, `lift.cpp`) and the toggle color sensor's hue thresholds (`toggle.cpp`) — need tuning against the real robot/toggles.
- Anti-tip constants in `main.cpp` (tip angle, correction direction) — unverified, test carefully before relying on them.
