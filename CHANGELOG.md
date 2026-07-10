# Changelog

All notable changes to this project are documented here. This project follows
[Semantic Versioning](https://semver.org/).

## [1.6.0] - 2026-07-10

### Added
- `clockwork::TrapezoidalProfile`, a speed-up / cruise / slow-down motion
  profile. Give it a distance, top speed, and acceleration; it hands back the
  target velocity and position at any time. Falls back to a triangle profile
  when the distance is too short to reach top speed, and supports reverse moves.
- `clockwork::FlywheelController`, a feedforward-plus-P velocity controller for
  a flywheel or any wheel you want holding an RPM. Output is clamped to one
  direction, and it recovers after a shot drags the speed down.
- `Motion::turnToHeading(heading, ...)`, turn to an absolute field heading with
  the chassis's tuned turn PID (the absolute counterpart to `turnBy`).
- `Motion::moveToPoint(x, y, ...)`, a blocking drive to a field point using
  lemlib's motion profiling, with an optional backwards mode.
- `clockwork::signedForwardDistance(...)` in a new `geometry.hpp`, the exact
  forward-progress projection `driveDistance` uses, pulled out so it can be
  tested on its own.

### Changed
- `driveDistance` now calls `signedForwardDistance` instead of an inline copy of
  the same math, so the shipped code and the tested helper are one and the same.
- Host test suite expanded to a stress pass: integral windup limits, output
  clamps under extreme input, convergence from both directions, slew never
  overshooting a jumping target, curve bounds and deadband edges, the geometry
  conventions, and every trapezoidal-profile and flywheel invariant. 700+ checks.

## [1.5.0] - 2026-07-09

### Added
- `clockwork::Pneumatics`, a wrapper for a pneumatic piston (an ADI digital-out
  solenoid). Tracks its own state so you get `extend()`, `retract()`, `set()`,
  `toggle()`, and `extended()` instead of loose `set_value` calls.
- `clockwork::Toggle`, a latch that flips a boolean on the released-to-pressed
  edge of a button. Handles the edge detection itself, so one press toggles once.
  Pairs with `Pneumatics` for one-button clamps and speed modes.
- `clockwork::joystickCurve(input, curve, deadband)`, exponential joystick
  shaping with a deadband, for finer low-speed driver control.
- Host tests extended to cover `Toggle` and `joystickCurve` (29 checks total).

### Changed
- Rewrote the header comments across the whole library in a plainer, more
  human voice. No API or behavior change.

## [1.4.0] - 2026-07-08

### Added
- `clockwork::AutonSelector`, a controller-driven autonomous selector. Register
  routines by name, scroll them with the D-pad, see the pick on the controller
  and brain screens, and `run()` the choice in `autonomous()`. Needs only core
  PROS (no liblvgl / LLEMU).
- `clockwork::SlewRateLimiter`, a small rate limiter that ramps a value toward a
  target by a fixed step per call. Smooths driver throttle (no wheelies or
  brownouts) or eases a flywheel up to speed.
- Host-side test suite for the pure-logic classes (`PIDController`,
  `SlewRateLimiter`). Runs on your computer with `make test` or `test/run.sh`,
  no PROS install and no V5 brain required.

## [1.3.0] - 2026-07-07

### Added
- `clockwork::PIDController`, a standalone, reusable PID controller for any
  subsystem lemlib does not drive (arm, lift, flywheel, wall-align). Includes
  anti-windup (integral cap + zero-cross reset), an output clamp, and a
  `settled()` helper. The README ships an in-depth, plain-language guide to
  what kP / kI / kD actually do and how to tune them.
- `Roller::antiJam(reversePower, reverseMs, jamHoldMs, velThreshold)`
  non-blocking anti-jam. Call it once per loop after commanding the intake; it
  auto-reverses a short burst when a jam is detected, then resumes the previous
  command. Works in both opcontrol and autonomous.
- `Motion::driveDistance` now takes an optional `driveKp` parameter (default
  `8.0`) so the distance gain, previously a hidden internal constant, can be
  tuned per robot.

### Fixed
- Straight-drive heading correction (`driveFullThenSlow`, `driveDistance`,
  `driveTimed`, `driveUntilStalled`) now clamps the turn term. Previously a
  large transient heading error produced an oversized turn command that
  `arcade()` desaturated by stealing throttle, so the robot could nearly stop
  and pivot instead of driving through the disturbance. Forward motion now
  always keeps priority.

## [1.2.0] - 2026-07-05

### Added
- `Motion::turnBy(degrees, timeoutMs, maxSpeed)`, relative turn using the
  chassis's tuned angular controller (blocking).
- `Motion::driveUntilStalled(power, timeoutMs, headingKp)`, drive until the
  robot stalls against a wall/obstacle, detected from odometry; returns whether
  it stalled. Useful for wall alignment before an odom reset.
- `Roller::hold()`, set HOLD brake mode and brake in place.
- `Roller::stalled(velThreshold)`, query stall state to build anti-jam logic.

## [1.1.0] - 2026-07-05

### Added
- `Motion::driveDistance(dist, maxSpeed, timeoutMs, headingKp, settleRange)`
  relative straight drive with proportional slowdown and heading hold.
- `Motion::driveTimed(ms, speed, headingKp)`, open-loop timed drive holding
  heading.
- `Roller`, motor-group wrapper (`in/out/stop/spin/pulse`) that replaces the
  repeated `motor1.move(x); motor2.move(x);` pattern.

## [1.0.0] - 2026-07-04

### Added
- Initial release.
- `Motion::driveFullThenSlow(fullDist, slowDist, slowSpeed, timeoutMs,
  fullSpeed, headingKp)`, two-phase straight drive (full speed then decelerate)
  with a P heading hold.
- PROS template packaging + `depot.json` for `pros c add-depot`.

[1.6.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.6.0
[1.5.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.5.0
[1.4.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.4.0
[1.3.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.3.0
[1.2.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.2.0
[1.1.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.1.0
[1.0.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.0.0
