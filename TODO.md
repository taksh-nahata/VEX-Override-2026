# What's left — Override 2026

Plain-language status, updated 2026-09-18. "Code" items need a programmer; everything else is hardware/testing anyone on the team can do. Coders: every tunable number in the actual code is tagged `TODO(tune)`, `TODO(verify)`, `TODO(mechanical)`, or `TODO(missing)` — `grep -rn "TODO("` in `src/`/`include/` for the exact list this doc summarizes.

## Done
- All 9 motor/sensor ports wired and confirmed correct.
- All motor spin directions bench-tested and fixed (drivetrain, lift, toggle spinner, odometry wheel).
- Claw tested and working.
- Drivetrain and lift both confirmed driving correctly.
- Lift has: a floor limit (won't drive below where it started), an automatic ceiling limit (stops before the gear cartridge skips at the top — no calibration needed, it just senses it), and a "landed on something" stop while lowering.
- Physical anti-tip bar — in progress.
- Custom on-screen menu at boot: real team logo + our own buttons for picking the autonomous routine, replacing the old generic-looking one. Builds and links clean — **still needs someone to actually upload and look at the brain screen to confirm it works right**, same as any other change; this one just had a rockier road getting there (see below).
- Controller rumble on key events (pin landed, hit ceiling, red detected) and an endgame warning at the 20-second mark — the brain's screen isn't visible to the driver mid-match, so this is the feedback that actually reaches them.
- Code is backed up on GitHub, with setup instructions for anyone joining the team.

## Found and fixed something worth knowing about the setup
- The screen-drawing library instructions (headers) didn't match the actual compiled screen-drawing code — turned out to be leftover from when EZ-Template was set up: the compiled library got copied over from last year's project, but its matching instructions never did. Found this by comparing files directly, fixed it by copying the correct matching instructions over too (they were sitting right there in last year's project). The custom logo + button menu at boot is built on the correct, fully-matched version now — no workarounds left in the code.

## Known mechanical issue — top priority, blocks a lot of the software tuning below
- **Left lift motor's gear is seated one tooth off from the right side.** This is a real, physical mechanical mismatch, not a software bug — it's why the lift looks crooked, and likely why the "pin landed" sensor fires randomly (the software keeps straining to sync two sides that physically can't agree, which shows up as extra current). Needs the gear pulled and reseated correctly. Software can't fix this, and the current-based tuning items below will keep giving unreliable numbers until it's fixed.

## Quick bench tests (a few minutes each, no coding — just watch the brain screen)
- **Tip-testing**: with a spotter holding the robot (lift LOW to start, anti-tip bar as backup), gently tilt it and watch the screen to figure out (a) how tilted is "actually about to tip" and (b) whether the auto-correction drives the right way. Our best guess says the direction is probably already correct, but "probably" isn't good enough to trust blind.
- **Pin-landing / ceiling sensor tuning**: lower the lift onto a real stack, and separately raise it to its mechanical top, watching the current numbers on screen each time — do this *after* the gear is fixed, since a crooked lift gives misleading numbers.
- **Measure the lift's full height** (as an encoder number, not inches) so the anti-tip speed-limiting scales correctly.
- **Double check wheel size (2.75") and gear ratio** are exactly right — sounded right in testing, just worth confirming precisely.

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
