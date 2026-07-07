# CLOCKWORK

![version](https://img.shields.io/badge/version-1.3.0-blue)
![platform](https://img.shields.io/badge/platform-VEX%20V5-red)
![PROS](https://img.shields.io/badge/PROS-kernel%20%5E4.2.1-orange)
![LemLib](https://img.shields.io/badge/depends-LemLib-green)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

A small, focused PROS library of motion, intake, and control primitives for the
VEX V5, built on top of [LemLib](https://lemlib.readthedocs.io/). Install it as
a PROS template and call clean helpers from your autonomous and driver code — no
copying source files between projects.

Created for V5RC team 2360C.

**New to PID / gains / "what is kP"? Jump straight to
[Understanding gains](#understanding-gains-kp-ki-kd--the-plain-english-guide).
It explains every tuning number in this library in plain language.**

---

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick start](#quick-start)
- [Understanding gains (kP, kI, kD) — the plain-English guide](#understanding-gains-kp-ki-kd--the-plain-english-guide)
- [API reference](#api-reference)
  - [`Motion`](#clockworkmotion)
  - [`Roller`](#clockworkroller)
  - [`PIDController`](#clockworkpidcontroller)
- [Tuning cheat sheet](#tuning-cheat-sheet)
- [Building & releasing](#building--releasing)

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
| `Roller::stalled`           | Query stall state |
| **`Roller::antiJam`**       | **Non-blocking auto-reverse when the intake jams, then resume** |
| **`PIDController`**         | **Reusable PID for any subsystem lemlib doesn't drive (arm, lift, flywheel)** |

Everything is generic and owns nothing: `Motion` wraps a `lemlib::Chassis*`,
`Roller` wraps a `pros::MotorGroup*`, and `PIDController` is standalone. You
keep full control of your own devices.

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
pros c fetch clockwork@1.3.0.zip
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

void opcontrol() {
    while (true) {
        intake.in();       // drive the intake
        intake.antiJam();  // clears jams by itself, never blocks
        pros::delay(20);
    }
}
```

More end-to-end routines: **[EXAMPLES.md](EXAMPLES.md)**.

---

## Understanding gains (kP, kI, kD) — the plain-English guide

Every "Kp" in this library is one idea: **how hard should the robot push to fix
a mistake?** You do not need any control-theory background. This section explains
it once, in words, and then everything else clicks.

### The one idea behind all of it

A controller is a loop that runs many times a second and asks one question:

> *How far am I from where I want to be?*

That gap is called the **error**. For a drive it might be "12 inches short of the
target." For an arm it might be "30 degrees below where I want it." The
controller turns that error into a motor power, waits a few milliseconds, checks
again, and repeats until the error is basically zero.

The only question is *how* it turns error into power. That is what the gains
control.

### kP — the proportional gain (the main dial)

**Power = kP × error.** The bigger the mistake, the harder it pushes; as it gets
close, it eases off automatically. `kP` is just the multiplier.

Think of a spring pulling the robot toward the target. `kP` is the stiffness of
that spring.

- **Too low:** weak spring. The robot creeps in slowly, or stops *short* of the
  target because there's not enough push left to finish (an arm sags below where
  you wanted it).
- **Too high:** violent spring. The robot races in and *overshoots*, then
  overshoots the other way — it oscillates or shakes.
- **Just right:** reaches the target quickly with only a tiny overshoot.

**`kP` is the one you tune first, and it does 90% of the work.**

In this library, `headingKp` (default `2.0`) and `driveKp` (default `8.0`) are
both pure `kP` dials: `headingKp` is how hard the robot corrects a drift in the
direction it's facing; `driveKp` is how hard it pushes toward a distance target.

### kD — the derivative gain (the brake)

A stiff `kP` overshoots because it's still pushing hard right up until it reaches
the target, so momentum carries it past. `kD` fixes that. It watches **how fast
the error is shrinking** and pushes *back* against fast approaches — it's a shock
absorber.

- Adds a braking force that grows the faster you're closing in.
- Lets you run a higher `kP` (fast) without the overshoot (sloppy).
- **Too high:** it fights every tiny sensor wiggle and the mechanism buzzes /
  gets jittery.

**Tune `kD` second, after `kP`, to kill the shake `kP` left behind.**

### kI — the integral gain (the closer, use sparingly)

Sometimes the robot settles *near* the target but never quite reaches it — a
small steady offset. Classic case: an arm held up against gravity, where `kP`'s
push at a tiny error isn't quite enough to hold it exactly level. `kI` fixes that
last sliver.

It **adds up** all the leftover error over time. As long as any error remains,
`kI`'s contribution keeps growing until it's finally strong enough to close the
gap.

- Only cures a small, *persistent* offset. It does nothing for speed or
  overshoot — that's `kP` and `kD`.
- **Keep it tiny** — often 10× to 100× smaller than `kP`.
- **Too high:** it "winds up" (over-accumulates) and causes slow, lazy
  oscillation.
- Many mechanisms work great with **`kI = 0`.** Start there.

`clockwork::PIDController` guards against wind-up automatically: it caps the
accumulated sum (`integralCap`) and clears it whenever the error crosses zero, so
a big move can't leave a huge leftover push waiting to overshoot.

### The tuning recipe (do this in order)

1. Set `kI = 0` and `kD = 0`.
2. Raise **`kP`** until the mechanism reaches the target quickly and overshoots
   just a little. If it shakes hard, you've gone too far — back off.
3. Raise **`kD`** until that overshoot/shake is smoothed out. If it starts
   buzzing, back off.
4. *Only if* it still stops slightly short of the target, add a **tiny `kI`**
   (start ~1/50th of `kP`) until it closes the gap. Otherwise leave `kI = 0`.

> Golden rule: **P, then D, then maybe I.** Change one gain at a time and watch
> what it does before touching the next.

### Symptom → which gain

| What you see | Likely cause | Do this |
|--------------|--------------|---------|
| Reaches target too slowly / stops short | `kP` too low | Raise `kP` |
| Overshoots then oscillates | `kP` too high | Lower `kP`, or add `kD` |
| Fast approach but shakes at the end | Needs damping | Add / raise `kD` |
| Buzzes or jitters constantly | `kD` too high | Lower `kD` |
| Settles just below/short of target forever | Steady offset | Add a tiny `kI` |
| Slow lazy wobble that won't die | `kI` wind-up | Lower `kI`, or set it to 0 |

### One thing that matters: loop timing

A controller assumes it runs at a **steady rhythm**. Always put a fixed delay in
your loop (`pros::delay(10)` is typical) and keep it the same. If you change the
loop delay, your tuned gains change meaning and you'll need to re-tune. This is
why every example here ends its loop with a `pros::delay`.

---

## API reference

### `clockwork::Motion`

```cpp
explicit Motion(lemlib::Chassis* chassis);
```

All drive helpers hold the starting heading with a P controller (`headingKp`) and
bypass the driver curve. Distance-based helpers measure from the pose at the call
site, so the chassis must have a valid pose first (`setPose`). Pass negative
speeds / distances to run in reverse.

| Method | Signature |
|--------|-----------|
| `driveFullThenSlow` | `(float fullDist, float slowDist, int slowSpeed, int timeoutMs, int fullSpeed = 127, float headingKp = 2.0f)` |
| `driveDistance`     | `(float dist, int maxSpeed = 127, int timeoutMs = 3000, float headingKp = 2.0f, float settleRange = 1.0f, float driveKp = 8.0f)` |
| `driveTimed`        | `(int ms, int speed, float headingKp = 2.0f)` |
| `turnBy`            | `(float degrees, int timeoutMs = 1500, int maxSpeed = 127)` |
| `driveUntilStalled` | `(int power, int timeoutMs = 3000, float headingKp = 2.0f) → bool` |

- **`driveFullThenSlow`** — full speed for `fullDist` inches, then `slowSpeed`
  for the next `slowDist` inches. Fast approach, soft arrival.
- **`driveDistance`** — drive `dist` inches, slowing proportionally near the
  target; settles within `settleRange` inches or times out. `driveKp` sets how
  hard it pushes per inch remaining (raise for snappier, lower if it overshoots).
- **`driveTimed`** — apply `speed` for `ms` milliseconds, then stop.
- **`turnBy`** — turn `degrees` relative to the current heading (+ clockwise)
  using the chassis's own angular PID. Blocking.
- **`driveUntilStalled`** — drive at `power` until the robot stops moving
  (wall/obstacle) or times out. Returns `true` if it stalled, `false` if it
  timed out. Great for wall alignment before an odom reset.

**`headingKp`** and **`driveKp`** are the `kP` gains explained
[above](#understanding-gains-kp-ki-kd--the-plain-english-guide). Defaults are
sane starting points; tune on your own robot.

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
| `antiJam(int reversePower = 127, int reverseMs = 200, int jamHoldMs = 150, double velThreshold = 5.0) → bool` | non-blocking auto-reverse on jam |

**`antiJam`** is the recommended way to keep an intake clear. Call it once per
loop *after* commanding the intake (`in`/`out`/`spin`). If the intake is
commanded to move but its velocity stays under `velThreshold` for longer than
`jamHoldMs`, it reverses at `reversePower` for `reverseMs` to clear the jam, then
resumes the previous command — all without blocking your loop. Returns `true`
while a clearing burst is in progress. Does nothing while the intake is stopped.

```cpp
while (true) {
    intake.in();
    intake.antiJam();      // auto-clears jams; resumes intaking on its own
    pros::delay(20);
}
```

### `clockwork::PIDController`

```cpp
PIDController(float kP, float kI, float kD,
              float integralCap = 0.0f, float outputCap = 0.0f);
```

A standalone PID for any subsystem lemlib doesn't drive — an arm, a lift, a
flywheel, a wall-align. See the
[gains guide](#understanding-gains-kp-ki-kd--the-plain-english-guide) for what
the three numbers mean and how to tune them.

| Method | Description |
|--------|-------------|
| `update(float error) → float` | Call every loop with `target - measured`; returns a motor power (clamped to `outputCap` if set) |
| `reset()` | Forget history (integral sum + last error). Call at the start of a new move |
| `setGains(float kP, float kI, float kD)` | Swap gains at runtime (e.g. loaded vs. empty) |
| `settled(float tolerance, float stillness = 1.0f) → bool` | True when the error is within `tolerance` **and** barely changing — i.e. actually arrived, not just passing through |

Constructor extras:
- **`integralCap`** — absolute limit on the accumulated integral (anti-windup).
  `0` disables it.
- **`outputCap`** — absolute limit on the returned power. **Pass `127`** so
  `update()` never returns an out-of-range motor command. `0` disables it.

```cpp
// arm: kP=0.9, kI=0, kD=4, integral cap 50, output clamped to motor range
clockwork::PIDController armPid(0.9f, 0.0f, 4.0f, 50.0f, 127.0f);

void moveArmTo(float targetDeg) {
    armPid.reset();
    while (!armPid.settled(2.0f)) {
        float measured = armSensor.get_position() / 100.0f;
        arm.move(armPid.update(targetDeg - measured));
        pros::delay(10); // steady loop timing — see the gains guide
    }
    arm.brake();
}
```

---

## Tuning cheat sheet

- **`headingKp`** (default `2.0`) — power per degree of heading error during
  straight drives. Robot drifts off heading → raise it. Robot wags side to side
  → lower it. (The turn correction is internally capped so a big bump can't
  hijack the drive — forward motion always wins.)
- **`driveKp`** (default `8.0`, `driveDistance` only) — power per inch of
  remaining distance. Arrives too slowly → raise it. Overshoots the target →
  lower it.
- **`settleRange`** (default `1.0` in) — how close counts as "arrived." Tighten
  for precision, loosen if it hunts around the target.
- **`driveUntilStalled`** treats "stopped" as very little pose movement over a
  short window. Fine on a normal drivetrain; if your odom is noisy, give it a
  longer timeout.
- **`PIDController`** — follow the recipe in the gains guide: **P, then D, then
  maybe I.**

All gains here are starting points, not tuned for any specific robot. Tune on
the field.

---

## Building & releasing

Requires the PROS toolchain (`pros` + `arm-none-eabi` on your `PATH`).

```bash
pros make            # build the project + bin/clockwork.a
pros make template   # package clockwork@<version>.zip
```

To cut a release: bump `VERSION` in the `Makefile`, `pros make template`, attach
the zip to a GitHub release, and add an entry to `depot.json`.

The template ships only the public headers (`include/clockwork/*.hpp`) and the
compiled archive (`firmware/clockwork.a`).

---

## Versioning & changelog

Semantic versioning. See **[CHANGELOG.md](CHANGELOG.md)**.

## Contributing

See **[CONTRIBUTING.md](CONTRIBUTING.md)**.

## License

MIT — see [LICENSE](LICENSE).
