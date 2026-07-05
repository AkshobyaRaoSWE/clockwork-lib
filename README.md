# CLOCKWORK

![version](https://img.shields.io/badge/version-1.2.0-blue)
![platform](https://img.shields.io/badge/platform-VEX%20V5-red)
![PROS](https://img.shields.io/badge/PROS-kernel%20%5E4.2.1-orange)
![LemLib](https://img.shields.io/badge/depends-LemLib-green)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

A small, focused PROS library of motion and intake primitives for the VEX V5,
built on top of [LemLib](https://lemlib.readthedocs.io/). Install it as a PROS
template and call clean helpers from your autonomous routines — no copying
source files between projects.

Created for V5RC team 2360C.

---

## Features

| Helper | What it does |
|--------|--------------|
| `Motion::driveFullThenSlow` | Two-phase straight drive: full speed, then decelerate to a slower speed |
| `Motion::driveDistance`     | Relative straight drive with proportional slowdown + heading hold |
| `Motion::driveTimed`        | Open-loop timed drive holding heading (ram / square-up) |
| `Motion::turnBy`            | Relative turn using the chassis's tuned angular controller |
| `Motion::driveUntilStalled` | Drive until the robot stalls against a wall (odometry-based) |
| `Roller::in/out/stop/spin`  | Readable intake/roller control — no more `motor1.move(); motor2.move();` |
| `Roller::pulse`             | Timed burst then stop (blocking) |
| `Roller::hold`              | Actively brake-hold position |
| `Roller::stalled`           | Query stall state to build anti-jam logic |

Everything is generic: `Motion` wraps a `lemlib::Chassis*`, `Roller` wraps a
`pros::MotorGroup*`. Neither owns the pointer.

---

## Requirements

- A PROS project (kernel `^4.2.1`)
- **LemLib** installed in that project (`pros c install LemLib`)

---

## Installation

### From the depot (recommended)

```bash
pros c add-depot clockwork https://raw.githubusercontent.com/AkshobyaRaoSWE/clockwork-lib/main/depot.json
pros c apply clockwork          # installs the latest version
```

### From a release zip

```bash
# download clockwork@x.y.z.zip from the Releases page, then:
pros c fetch clockwork@1.2.0.zip
pros c apply clockwork
```

> **Gotcha:** `pros c apply <path-to-zip>` does *not* read a filesystem path —
> it parses it as a `name@version` query. Always `pros c fetch <zip>` first,
> then `pros c apply clockwork`.

To upgrade later: `pros c apply clockwork` again after the depot refreshes.

---

## Quick start

```cpp
#include "clockwork/clockwork.hpp"

// `chassis` and `intake_motors` are your own configured globals.
clockwork::Motion motion(&chassis);
clockwork::Roller intake(&intake_motors);

void autonomous() {
    chassis.setPose(0, 0, 0);

    intake.in();                          // start intaking
    motion.driveDistance(30, 110, 2000);  // drive 30 in, heading-held
    motion.turnBy(90);                    // turn 90° clockwise
    motion.driveFullThenSlow(18, 6, 40, 1500); // fast, then gentle arrival
    intake.pulse(-127, 300);              // eject for 300 ms

    if (motion.driveUntilStalled(70, 1200)) // ram the wall
        chassis.setPose(0, 0, 0);           // ...and reset odom if we hit it
}
```

More end-to-end routines: **[EXAMPLES.md](EXAMPLES.md)**.

---

## API reference

### `clockwork::Motion`

```cpp
explicit Motion(lemlib::Chassis* chassis);
```

All drive helpers hold the starting heading with a P controller and bypass the
driver curve. Distance-based helpers measure from the pose at the call site, so
the chassis must have a valid pose first (`setPose`). Pass negative speeds /
distances to run in reverse.

| Method | Signature |
|--------|-----------|
| `driveFullThenSlow` | `(float fullDist, float slowDist, int slowSpeed, int timeoutMs, int fullSpeed = 127, float headingKp = 2.0f)` |
| `driveDistance`     | `(float dist, int maxSpeed = 127, int timeoutMs = 3000, float headingKp = 2.0f, float settleRange = 1.0f)` |
| `driveTimed`        | `(int ms, int speed, float headingKp = 2.0f)` |
| `turnBy`            | `(float degrees, int timeoutMs = 1500, int maxSpeed = 127)` |
| `driveUntilStalled` | `(int power, int timeoutMs = 3000, float headingKp = 2.0f) → bool` |

- **`driveFullThenSlow`** — full speed for `fullDist` inches, then `slowSpeed`
  for the next `slowDist` inches. Fast approach, soft arrival.
- **`driveDistance`** — drive `dist` inches, slowing proportionally near the
  target; settles within `settleRange` inches or times out.
- **`driveTimed`** — apply `speed` for `ms` milliseconds, then stop.
- **`turnBy`** — turn `degrees` relative to the current heading (+ clockwise)
  using the chassis's own angular PID. Blocking.
- **`driveUntilStalled`** — drive at `power` until the robot stops moving
  (wall/obstacle) or times out. Returns `true` if it stalled, `false` if it
  timed out. Great for wall alignment before an odom reset.

### `clockwork::Roller`

```cpp
explicit Roller(pros::MotorGroup* motors, int defaultPower = 127);
```

| Method | Description |
|--------|-------------|
| `in()`  | spin inward at the default power |
| `out()` | spin outward at the default power |
| `stop()` | command 0 |
| `spin(int power)` | explicit signed power, −127..127 |
| `pulse(int power, int ms)` | spin for `ms`, then stop (blocking) |
| `hold()` | set HOLD brake mode and brake in place |
| `stalled(double velThreshold = 5.0) → bool` | true if velocity is under `velThreshold` RPM |

```cpp
// simple anti-jam
intake.in();
if (intake.stalled()) intake.pulse(-127, 150); // kick back on a jam
```

---

## Tuning notes

- **`headingKp`** (default `2.0`) — power per degree of heading error during
  straight drives. Raise it if the robot drifts off heading; lower it if it
  oscillates.
- **`driveDistance`** uses an internal distance gain and a small minimum power
  to overcome static friction near the target; adjust `settleRange` for how
  tightly it must arrive.
- **`driveUntilStalled`** treats "stopped" as very little pose movement over a
  short window. Fine on a normal drivetrain; if your odom is noisy, give it a
  longer timeout.

All motion gains are unverified on any specific robot — tune on the field.

---

## Building & releasing

Requires the PROS toolchain (`pros` + `arm-none-eabi` on your `PATH`).

```bash
pros make            # build the project + bin/clockwork.a
pros make template   # package clockwork@<version>.zip
```

To cut a release: bump `VERSION` in the `Makefile`, `pros make template`,
attach the zip to a GitHub release, and add an entry to `depot.json`.

The template ships only the public headers (`include/clockwork/*.hpp`) and the
compiled archive (`firmware/clockwork.a`).

---

## Versioning & changelog

Semantic versioning. See **[CHANGELOG.md](CHANGELOG.md)**.

## Contributing

See **[CONTRIBUTING.md](CONTRIBUTING.md)**.

## License

MIT — see [LICENSE](LICENSE).
