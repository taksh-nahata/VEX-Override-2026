# What's left — Override 2026

Plain-language status, updated 2026-09-12. "Code" items need a programmer; everything else is hardware/testing anyone on the team can do.

## Done
- All 9 motor/sensor ports wired and confirmed correct.
- All motor spin directions bench-tested and fixed (drivetrain, lift, toggle spinner).
- Claw tested and working.
- Drivetrain and lift both confirmed driving correctly (manual control).
- Code is backed up on GitHub, with setup instructions for anyone joining the team.

## Quick bench tests (a few minutes each, no coding — just watch the brain screen)
- **Odometry direction check**: push the robot to the right by hand while it's on, watch the number on the brain screen — should count up. If it counts down, tell the coder to flip one setting.
- **Tip-testing**: with a spotter holding the robot (lift LOW to start), gently tilt it and watch the numbers on screen to figure out (a) how tilted is "actually about to tip" and (b) whether the auto-correction drives the right way or the wrong way. Doing this now, safely, is much better than finding out mid-match.
- **Pin-landing sensor tuning**: lower the lift onto a real stack a bunch of times and watch the current numbers on screen, so the coder can set the "it just landed" threshold correctly.
- **Measure the lift's full height** (as an encoder number, not inches) so the anti-tip speed-limiting scales correctly.
- **Double check wheel size (2.75") and gear ratio** are exactly right — sounded right in testing, just worth confirming precisely.

## Decisions the team needs to make (not urgent)
- **Aligner push sensor** — add a cheap limit switch to the aligner standoffs so the robot can auto-stop the instant it's aligned with the goal, instead of driving in by feel? Recommended, cheap, simple — just needs someone to say yes and give a port number.
- **Should pin release become automatic?** Right now the driver still presses a button to actually drop the pin, even though the lift auto-stops when it senses the pin has landed. Making release automatic too would save one more button press, but removes the driver's last visual check before an action that can't be undone. Worth revisiting once the landing-sensor is well-tested and trusted.

## Tuning that needs the finished, weighed robot (normal, expected, not a problem)
- Drivetrain turning/driving PID numbers — inherited placeholders from last year's different robot.
- Lift height PID numbers — never tuned for this year's actual lift.
- Toggle color sensor thresholds — never tuned against the real toggles under real lighting.

## Not written yet
- **Actual autonomous routines.** The framework for 3 auton slots exists, but the actual scoring paths/moves are all empty right now — someone needs to write these once driving and placement are dialed in.

## Ideas floated, not committed to
- Toggle auto-spin-to-alliance-color — held off because we're not 100% sure how the toggle mechanism physically works (spin vs. lift-and-flip). Check VEX's official field build video first.
- Using a $100 AI Vision sensor to auto-aim at goals via AprilTags — a real trend this season, but a real purchase, not decided.
- Auto-drive-to-midfield in the last 20 seconds — cheap to add once autons exist and odometry is tuned.
