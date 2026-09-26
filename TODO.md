# What's left — Override 2026

Plain-language status, updated 2026-09-25. "Code" items need a programmer; everything else is hardware/testing anyone on the team can do. Coders: every tunable number in the actual code is tagged `TODO(tune)`, `TODO(verify)`, `TODO(mechanical)`, or `TODO(missing)` — `grep -rn "TODO("` in `src/`/`include/` for the exact list this doc summarizes.

## Done
- All motor/sensor ports wired and confirmed correct — several moved around 2026-09-20/25 to make room for the new lift motor + rotation sensor (drivetrain right-back motor: 20 → 15; lift motor: 20 → 1; lift rotation sensor: 12 → 8; see "Lift rebuild" below).
- All motor spin directions bench-tested and fixed (drivetrain, toggle spinner, odometry wheel, and now the new single lift motor + its rotation sensor — both confirmed 2026-09-25, and they needed different fixes: the motor's direction was already right, the rotation sensor's was backwards).
- Claw tested and working.
- Drivetrain confirmed driving correctly. Wheel size (2.75" drive, 2" odom) and the drive's external gear ratio (36T motor / 48T wheel) are now all confirmed exactly, not just "sounded right."
- **Lift rebuilt from 2 motors to 1** (2026-09-20) — see "Lift rebuild" below, this replaced the old crooked-lift problem instead of working around it.
- Lift has: a floor limit (won't drive below where it started), an automatic ceiling limit (stops before the gear cartridge skips at the top — no calibration needed, it just senses it), a "landed on something" stop while lowering, and an active idle hold (2026-09-25) — it now servos against gravity to stay exactly where you left it instead of just relying on the motor's own brake, which turned out not to be strong enough on its own for one motor carrying the whole arm.
- Physical anti-tip bar — in progress.
- Custom on-screen menu at boot: real team logo + our own buttons for picking the autonomous routine, replacing the old generic-looking one. Builds and links clean — **still needs someone to actually upload and look at the brain screen to confirm it works right**, same as any other change; this one just had a rockier road getting there (see below).
- Controller rumble on key events (pin landed, hit ceiling, red detected) and an endgame warning at the 20-second mark — the brain's screen isn't visible to the driver mid-match, so this is the feedback that actually reaches them.
- Code is backed up on GitHub, with setup instructions for anyone joining the team.
- **Drivetrain PID tuner** (EZ-Template's built-in one) wired up to the controller — X turns it on/off, B runs a test drive+turn move, live values show on the brain. For actually tuning the drive/turn numbers.
- **SD card logging** — every drive/lift number gets written to a file on the SD card, 20 times a second, the whole time the robot's on. Lets us review exactly what happened after a weird glitch instead of trying to catch it live on the tiny brain screen.
- **Drivetrain/odometry calibration test moves** — A drives a fixed test distance (tape-measure the result and report it back), LEFT spins in place to auto-measure the odometry tracking wheel's mounting offset (no measuring needed for that one).

## Found and fixed something worth knowing about the setup
- The screen-drawing library instructions (headers) didn't match the actual compiled screen-drawing code — turned out to be leftover from when EZ-Template was set up: the compiled library got copied over from last year's project, but its matching instructions never did. Found this by comparing files directly, fixed it by copying the correct matching instructions over too (they were sitting right there in last year's project). The custom logo + button menu at boot is built on the correct, fully-matched version now — no workarounds left in the code.

## Lift rebuild (2026-09-20) — replaced the old mechanical issue, don't skip the direction check
The old 2-motor lift had a left gear seated one tooth off from the right, which made it look crooked and caused the current sensors to fire randomly no matter how the software was tuned — a real physical mismatch, not something code could fix. Rather than reseat that gear, the team switched to **1 motor** mounted on the second four-bar through a 1:6 external reduction (12T on the motor, 72T on the four-bar's shaft), plus a **rotation sensor** mounted on that same 72T shaft to read the arm's true angle directly instead of trusting the motor's encoder through the gear mesh. Confirmed on the bench: no more skipping.

This is brand new hardware, so:
- ~~Both the motor's direction and the rotation sensor's direction need a bench check with R1~~ — **done 2026-09-25**: motor was already correct, rotation sensor was backwards (read negative going up) and got flipped in code.
- Every number that was tuned against the old lift's "degrees" (floor tolerance, the anti-tip height cutoff) is now measuring something different — the rotation sensor sits past the new 1:6 reduction, so a degree now covers about 6x the arm movement it used to. These all need fresh real-world numbers, not leftover ones from before the rebuild.
- **Always power the robot on with the lift all the way down at true floor.** The rotation sensor zeros to wherever the lift physically is the moment the robot boots, not to any permanent reference — so the floor limit and the anti-tip height cutoff only mean what they're supposed to if "0" is consistently the real floor. This is a pit habit now, not a one-time setup step.

## Quick bench tests (a few minutes each, no coding — just watch the brain screen)
- **Tip-testing**: with a spotter holding the robot (lift LOW to start, anti-tip bar as backup), gently tilt it and watch the screen to figure out (a) how tilted is "actually about to tip" and (b) whether the auto-correction drives the right way. Our best guess says the direction is probably already correct, but "probably" isn't good enough to trust blind.
- **Pin-landing / ceiling sensor tuning**: lower the lift onto a real stack, and separately raise it to its mechanical top, watching the current number on screen (or the SD card log afterward) each time.
- **Measure the lift's full height** on the new rotation sensor's numbers (not carried over from the old lift) so the anti-tip speed-limiting scales correctly.
- **Odometry tracking wheel offset** — run the LEFT-button calibration spin (see "Done" above) and report the printed offset estimate back.

## Decisions the team needs to make (not urgent)
- **Aligner push sensor** — a cheap limit switch on the standoffs, but NOT to auto-stop the drivetrain (the standoffs hitting the goal already stop the robot on their own — cutting the motors on top of that could actually let the robot drift/settle away from the goal, since it runs in "coast" mode, not "hold"). The real value: use it to tell the driver "you're aligned" (so no more guessing), or to auto-trigger the next step. Still just an idea — needs someone to decide and give a port number.
- **Should pin release become automatic?** Right now the driver still presses a button to actually drop the pin, even though the lift auto-stops when it senses the pin has landed. Making release automatic too would save one more button press, but removes the driver's last visual check before an action that can't be undone. Worth revisiting once the landing-sensor is well-tested and trusted.

## Tuning that needs the finished, weighed, straightened robot (normal, expected, not a problem)
- Drivetrain turning/driving PID numbers — inherited placeholders from last year's different robot.
- Lift height PID numbers — never tuned for this year's actual lift.
- Toggle color sensor thresholds — never tuned against the real toggles under real lighting.

## Not written yet
- **Actual autonomous routines.** The framework for 3 auton slots exists, but the actual scoring paths/moves are all empty right now — someone needs to write these once driving and placement are dialed in.
- LQR for drivetrain control — flagged as worth investigating once we get to auton routines specifically (better fit for tracking precision than for anti-tip, which needs the physical bar more than fancier math).

## Ideas floated, not committed to
- Toggle auto-spin-to-alliance-color — held off because we're not 100% sure how the toggle mechanism physically works (spin vs. lift-and-flip). Check VEX's official field build video first.
- Using a $100 AI Vision sensor to auto-aim at goals via AprilTags — a real trend this season, but a real purchase, not decided.
- Auto-drive-to-midfield in the last 20 seconds — cheap to add once autons exist and odometry is tuned.
