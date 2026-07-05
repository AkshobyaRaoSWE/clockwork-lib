# Changelog

All notable changes to this project are documented here. This project follows
[Semantic Versioning](https://semver.org/).

## [1.2.0] — 2026-07-05

### Added
- `Motion::turnBy(degrees, timeoutMs, maxSpeed)` — relative turn using the
  chassis's tuned angular controller (blocking).
- `Motion::driveUntilStalled(power, timeoutMs, headingKp)` — drive until the
  robot stalls against a wall/obstacle, detected from odometry; returns whether
  it stalled. Useful for wall alignment before an odom reset.
- `Roller::hold()` — set HOLD brake mode and brake in place.
- `Roller::stalled(velThreshold)` — query stall state to build anti-jam logic.

## [1.1.0] — 2026-07-05

### Added
- `Motion::driveDistance(dist, maxSpeed, timeoutMs, headingKp, settleRange)` —
  relative straight drive with proportional slowdown and heading hold.
- `Motion::driveTimed(ms, speed, headingKp)` — open-loop timed drive holding
  heading.
- `Roller` — motor-group wrapper (`in/out/stop/spin/pulse`) that replaces the
  repeated `motor1.move(x); motor2.move(x);` pattern.

## [1.0.0] — 2026-07-04

### Added
- Initial release.
- `Motion::driveFullThenSlow(fullDist, slowDist, slowSpeed, timeoutMs,
  fullSpeed, headingKp)` — two-phase straight drive (full speed then decelerate)
  with a P heading hold.
- PROS template packaging + `depot.json` for `pros c add-depot`.

[1.2.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.2.0
[1.1.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.1.0
[1.0.0]: https://github.com/AkshobyaRaoSWE/clockwork-lib/releases/tag/v1.0.0
