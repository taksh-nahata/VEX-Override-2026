# What's left — Override 2026

Plain-language status, updated 2026-09-26. "Code" items need a programmer; everything else is hardware/testing anyone on the team can do. Coders: every tunable number in the actual code is tagged `TODO(tune)`, `TODO(verify)`, `TODO(mechanical)`, or `TODO(missing)` — `grep -rn "TODO("` in `src/`/`include/` for the exact list this doc summarizes.

## Done
- All motor/sensor ports wired and confirmed correct — several moved around 2026-09-20/25 to make room for the new lift motor + rotation sensor (drivetrain right-back motor: 20 → 15; lift motor: 20 → 1; lift rotation sensor: 12 → 8; see "Lift rebuild" below).
- All motor spin directions bench-tested and fixed (drivetrain, toggle spinner, odometry wheel, and now the new single lift motor + its rotation sensor — both confirmed 2026-09-25, and they needed different fixes: the motor's direction was already right, the rotation sensor's was backwards).
- Claw tested and working.
- Drivetrain confirmed driving correctly. Wheel size (2.75" drive, 2" odom) and the drive's external gear ratio (36T motor / 48T wheel) are now all confirmed exactly, not just "sounded right."
- **Lift rebuilt from 2 motors to 1** (2026-09-20) — see "Lift rebuild" below, this replaced the old crooked-lift problem instead of working around it.
- Lift has: an automatic ceiling limit (stops before the gear cartridge skips at the top — no calibration needed) and a "hit something solid" stop while lowering (a stack, or the true floor — same current-sensing check catches both; team decided 2026-09-25 this is good enough on its own, so the separate position-based floor limit was removed).
- Lift idle hold (2026-09-25): stays exactly where you left it by actively correcting gravity sag, using the motor's own brake for everything smaller than that so it still resists a hand-push normally (a first version ran the correction constantly and made the lift too easy to backdrive — fixed with a deadband).
- Physical anti-tip bar — in progress, still the real fix, everything below is a backstop underneath it.
- **Anti-tip made smarter (2026-09-26)**: it used to only look at how far the robot had already tilted. Now it also watches how *fast* it's tilting, so a hard, fast tip gets caught before it reaches the same angle a slow, harmless lean (like driving up a bump) would eventually reach on its own. Also added hysteresis so the speed cap doesn't flicker on and off if the robot is bouncing right at the threshold.
- Custom on-screen menu at boot: real team logo + our own buttons for picking the autonomous routine, replacing the old generic-looking one. Builds and links clean — **still needs someone to actually upload and look at the brain screen to confirm it works right**, same as any other change; this one just had a rockier road getting there (see below).
- Controller rumble on key events (pin landed, hit ceiling, target color reached) and an endgame warning at the 20-second mark — the brain's screen isn't visible to the driver mid-match, so this is the feedback that actually reaches them. The toggle target color (red/blue/yellow) now also shows continuously on the controller's own screen, not just for a moment right after pressing UP/Y.
- Code is backed up on GitHub, with setup instructions for anyone joining the team.
- **SD card logging** — every drive/lift number gets written to a file on the SD card, 20 times a second, the whole time the robot's on. Lets us review exactly what happened after a weird glitch instead of trying to catch it live on the tiny brain screen.
- **Lift height presets (2026-09-26)** — X/B/A send the lift straight to Pin 1 / Pin 2 / Pin 3 height (the pin going onto an empty goal / a goal with 1 pin / a goal with 2), instead of the driver eyeballing it with R1/R2 every time. Prints which one got picked to the controller screen. The three target heights are still just guesses — needs bench testing against the real stack heights (see below).
- **Drivetrain PID tuner and calibration test moves still exist** (`autons.cpp`) but aren't wired to a controller button right now — we freed up X/B/A/LEFT for the lift presets above. Easy to put back on the controller whenever we're back to tuning the drivetrain.

## Found and fixed something worth knowing about the setup
- The screen-drawing library instructions (headers) didn't match the actual compiled screen-drawing code — turned out to be leftover from when EZ-Template was set up: the compiled library got copied over from last year's project, but its matching instructions never did. Found this by comparing files directly, fixed it by copying the correct matching instructions over too (they were sitting right there in last year's project). The custom logo + button menu at boot is built on the correct, fully-matched version now — no workarounds left in the code.

## Lift rebuild (2026-09-20) — replaced the old mechanical issue, don't skip the direction check
The old 2-motor lift had a left gear seated one tooth off from the right, which made it look crooked and caused the current sensors to fire randomly no matter how the software was tuned — a real physical mismatch, not something code could fix. Rather than reseat that gear, the team switched to **1 motor** mounted on the second four-bar through a 1:6 external reduction (12T on the motor, 72T on the four-bar's shaft), plus a **rotation sensor** mounted on that same 72T shaft to read the arm's true angle directly instead of trusting the motor's encoder through the gear mesh. Confirmed on the bench: no more skipping.

This is brand new hardware, so:
- ~~Both the motor's direction and the rotation sensor's direction need a bench check with R1~~ — **done 2026-09-25**: motor was already correct, rotation sensor was backwards (read negative going up) and got flipped in code.
- Every number that was tuned against the old lift's "degrees" (floor tolerance, the anti-tip height cutoff) is now measuring something different — the rotation sensor sits past the new 1:6 reduction, so a degree now covers about 6x the arm movement it used to. These all need fresh real-world numbers, not leftover ones from before the rebuild.
- **Always power the robot on with the lift all the way down at true floor.** The rotation sensor zeros to wherever the lift physically is the moment the robot boots, not to any permanent reference — so the anti-tip height cutoff only means what it's supposed to if "0" is consistently the real floor. This is a pit habit now, not a one-time setup step.

## Quick bench tests (a few minutes each, no coding — just watch the brain screen)
- **Tip-testing**: with a spotter holding the robot (lift LOW to start, anti-tip bar as backup), gently tilt it slowly and watch the screen to find (a) how tilted is "actually about to tip" and (b) whether the auto-correction drives the right way — our best guess says the direction is already correct, but "probably" isn't good enough to trust blind. Then do it again fast (still safely, still spotted) to see if the new speed-based detection catches it earlier than the angle alone would have.
- **Pin-landing / ceiling sensor tuning**: lower the lift onto a real stack, and separately raise it to its mechanical top, watching the current number on screen (or the SD card log afterward) each time.
- **Measure the lift's full height** on the new rotation sensor's numbers (not carried over from the old lift) so the anti-tip speed-limiting scales correctly.
- ~~Odometry tracking wheel offset~~ — **done 2026-09-26**: measured -1.97in with the calibration spin (temporarily unbound from a button, see above), now in `ODOM_HORIZONTAL_OFFSET`. Landing angle also got a little straighter after slowing the test spin down — some of the leftover error was the IMU getting less accurate at higher spin speeds, some is just Turn PID's exit tolerance being loose (normal, part of the drivetrain tuning already planned).
- **Lift height presets** — press X, then B, then A, and see how close each one actually lands next to a real 1-pin/2-pin/3-pin stack. Report back what to change `PIN_1/2/3_HEIGHT_DEG` (`lift.cpp`) to.
- **Drivetrain drifts during an in-place turn instead of staying put** — noticed during the odometry spin test. Points at uneven weight or traction side-to-side (skid-steer pivots around its real center of mass/friction, not necessarily the geometric center) — worth a look before autons get built around multi-turn paths, since each turn will nudge the robot's real position a little.

## Decisions the team needs to make (not urgent)
- **Aligner push sensor** — a cheap limit switch on the standoffs, but NOT to auto-stop the drivetrain (the standoffs hitting the goal already stop the robot on their own — cutting the motors on top of that could actually let the robot drift/settle away from the goal, since it runs in "coast" mode, not "hold"). The real value: use it to tell the driver "you're aligned" (so no more guessing), or to auto-trigger the next step. Still just an idea — needs someone to decide and give a port number.
- **Should pin release become automatic?** Right now the driver still presses a button to actually drop the pin, even though the lift auto-stops when it senses the pin has landed. Making release automatic too would save one more button press, but removes the driver's last visual check before an action that can't be undone. Worth revisiting once the landing-sensor is well-tested and trusted.

## Tuning that needs the finished, weighed, straightened robot (normal, expected, not a problem)
- Drivetrain turning/driving PID numbers — inherited placeholders from last year's different robot.
- Lift height PID numbers — never tuned for this year's actual lift.
- Toggle color sensor thresholds — never tuned against the real toggles under real lighting.

## Not written yet
- LQR for drivetrain control — flagged as worth investigating once we get to auton routines specifically (better fit for tracking precision than for anti-tip, which needs the physical bar more than fancier math).

## First auto written (2026-09-26) — needs real field numbers before it'll work
`auton_button_1()` (`autons.cpp`) scores the preload, then does 2 Loader cycles for a 3-pin auto total, going for the 12-point auto bonus (easy — just outscore the other alliance's auto) rather than the 7-pin Autonomous Win Point (not realistic without an intake in 15 seconds — every grab needs precise alignment a claw can't do quickly). Uses the Pin 1/2/3 height presets from above, in order, as it stacks each pin.

Every drive distance and turn angle in it is a placeholder — **someone needs to pace out or measure the real distances from the starting tile to the goal and to the Loader** and report them back. Also still open:
- Does grabbing from the Loader need the lift at a specific height, or is floor height fine? If it needs its own height, that's a 4th preset the same way the pin ones work.
- Does the claw need to be open or closed to receive a pin from the Loader? Assumed open (same state it's in right after dropping a pin) — worth confirming on the real Loader.
- 15 seconds is tight for 3 full Loader cycles, especially with Drive/Turn PID still untuned. Time it for real once the distances are filled in — don't be surprised if it needs cutting to 2 pins (preload + 1 cycle) to actually fit.
- `auton_button_2()` and `auton_skills()` are still stubs — skills is its own game mode with different timing, not just a longer version of this, so it needs its own plan later.

## Ideas floated, not committed to
- Toggle auto-spin-to-alliance-color — held off because we're not 100% sure how the toggle mechanism physically works (spin vs. lift-and-flip). Check VEX's official field build video first.
- Using a $100 AI Vision sensor to auto-aim at goals via AprilTags — a real trend this season, but a real purchase, not decided.
- Auto-drive-to-midfield in the last 20 seconds — cheap to add once autons exist and odometry is tuned.
