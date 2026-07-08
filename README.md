# CLOCKWORK

![version](https://img.shields.io/badge/version-1.4.0-blue)
![platform](https://img.shields.io/badge/platform-VEX%20V5-red)
![PROS](https://img.shields.io/badge/PROS-kernel%20%5E4.2.1-orange)
![LemLib](https://img.shields.io/badge/depends-LemLib-green)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

A small, focused PROS library of motion, intake, and control helpers for the
VEX V5, built on top of [LemLib](https://lemlib.readthedocs.io/). The idea is
simple: install it once as a PROS template and call clean helpers from your
autonomous and driver code, instead of copy-pasting the same motion code into
every new project.

Written for V5RC team 2360C.

**Docs website:** https://akshobyaraoswe.github.io/clockwork-lib/

If PID and gains are new to you, or you have ever wondered what "kP" actually
means, start with [Understanding gains](#understanding-gains-kp-ki-kd). It
explains every tuning number in this library in plain words.

---

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick start](#quick-start)
- [Understanding gains (kP, kI, kD)](#understanding-gains-kp-ki-kd)
- [API reference](#api-reference)
  - [`Motion`](#clockworkmotion)
  - [`Roller`](#clockworkroller)
  - [`PIDController`](#clockworkpidcontroller)
  - [`AutonSelector`](#clockworkautonselector)
  - [`SlewRateLimiter`](#clockworkslewratelimiter)
- [Tuning cheat sheet](#tuning-cheat-sheet)
- [Testing](#testing)
- [Building and releasing](#building-and-releasing)

---

## Features

| Helper | What it does |
|--------|--------------|
| `Motion::driveFullThenSlow` | Drive straight fast, then ease down to a slow speed for a gentle arrival |
| `Motion::driveDistance`     | Drive a set number of inches, slowing near the target, holding heading |
| `Motion::driveTimed`        | Push at a fixed power for a fixed time (ram or square up) |
| `Motion::turnBy`            | Turn a relative number of degrees using the chassis's tuned turn PID |
| `Motion::driveUntilStalled` | Drive until the robot hits something, detected from odometry |
| `Roller::in/out/stop/spin`  | Readable intake control, instead of `motor1.move(); motor2.move();` |
| `Roller::pulse`             | Spin for a set time, then stop (blocking) |
| `Roller::hold`              | Actively brake-hold position |
| `Roller::stalled`           | Ask whether the intake is stalled |
| **`Roller::antiJam`**       | **Clears intake jams on its own, without blocking your loop** |
| **`PIDController`**         | **A reusable PID for anything LemLib doesn't drive: arm, lift, flywheel** |
| **`AutonSelector`**         | **Pick an autonomous routine from the controller before the match** |
| **`SlewRateLimiter`**       | **Ramp a value so a hard joystick shove can't wheelie or brown out** |

Nothing here owns your hardware. `Motion` borrows a `lemlib::Chassis*`, `Roller`
borrows a `pros::MotorGroup*`, and `PIDController` is standalone. You keep full
control of your own devices.

---

## Requirements

- A PROS project on kernel `^4.2.1`
- **LemLib** installed in that project (`pros c install LemLib`)

---

## Installation

### From the depot (recommended)

```bash
pros c add-depot clockwork https://raw.githubusercontent.com/AkshobyaRaoSWE/clockwork-lib/main/depot.json
pros c apply clockwork
```

### From a release zip

```bash
# download clockwork@x.y.z.zip from the Releases page, then:
pros c fetch clockwork@1.4.0.zip
pros c apply clockwork
```

One thing that trips people up: `pros c apply <path-to-zip>` does not read a file
path. It reads the argument as a `name@version` query. So always `fetch` the zip
first, then `apply clockwork`.

To upgrade later, run `pros c apply clockwork` again once the depot refreshes.

---

## Quick start

Point the helpers at your own `chassis` and intake motor group, then call them
from your routines.

```cpp
#include "clockwork/clockwork.hpp"

// chassis and intake_motors are your own configured globals.
clockwork::Motion motion(&chassis);
clockwork::Roller intake(&intake_motors);

void autonomous() {
    chassis.setPose(0, 0, 0);

    intake.in();                          // start intaking
    motion.driveDistance(30, 110, 2000);  // drive 30 in, heading held
    motion.turnBy(90);                    // turn 90 deg clockwise
    motion.driveFullThenSlow(18, 6, 40, 1500); // fast, then gentle arrival
    intake.pulse(-127, 300);              // eject for 300 ms

    if (motion.driveUntilStalled(70, 1200)) // ram the wall
        chassis.setPose(0, 0, 0);           // and reset odom if we hit it
}

void opcontrol() {
    while (true) {
        intake.in();       // drive the intake
        intake.antiJam();  // clears jams by itself, never blocks
        pros::delay(20);
    }
}
```

More complete routines are in **[EXAMPLES.md](EXAMPLES.md)**.

---

## Understanding gains (kP, kI, kD)

Every "Kp" in this library is really one question: **how hard should the robot
push to fix a mistake?** You do not need any control-theory background for this.
Read it once and the rest of the library makes sense.

### The one idea behind all of it

A controller is a loop that runs many times a second and asks one thing:

> How far am I from where I want to be?

That gap is the **error**. For a drive it might be "12 inches short of the
target." For an arm it might be "30 degrees too low." The controller turns that
error into a motor power, waits a few milliseconds, checks again, and repeats
until the error is basically zero. The gains decide how it turns error into
power.

### kP, the proportional gain (the main dial)

**Power = kP times error.** The bigger the mistake, the harder it pushes, and it
eases off on its own as it gets close. Picture a spring pulling the robot toward
the target, and kP is how stiff that spring is.

- **Too low:** weak spring. It creeps in slowly, or stops short of the target
  (an arm sags below where you wanted it).
- **Too high:** violent spring. It races in and overshoots, then overshoots the
  other way. It shakes.
- **Just right:** reaches the target quickly with only a tiny overshoot.

This is the one you tune first, and it does about 90% of the work. In this
library, `headingKp` (default `2.0`) and `driveKp` (default `8.0`) are both pure
kP dials. `headingKp` is how hard the robot corrects a drift in the direction it
is facing, and `driveKp` is how hard it pushes toward a distance target.

### kD, the derivative gain (the brake)

A stiff kP overshoots because it keeps pushing hard right up until it arrives, so
momentum carries it past. kD is the shock absorber. It watches how fast the error
is shrinking and pushes back against fast approaches.

- It adds a braking force that grows the faster you are closing in.
- It lets you run a higher kP (fast) without the overshoot (sloppy).
- Too high, and it fights every little sensor wiggle, so the mechanism buzzes.

Tune kD second, after kP, to kill the shake that kP left behind.

### kI, the integral gain (the closer, use sparingly)

Sometimes it settles near the target but never quite reaches it, a small steady
offset. The classic case is an arm held up against gravity, where kP's push at a
tiny error is not quite enough to hold it level. kI fixes that last sliver by
adding up the leftover error over time, so as long as any error remains, kI's
contribution keeps growing until it is finally strong enough to close the gap.

- It only cures a small, persistent offset. It does nothing for speed or
  overshoot, that is kP and kD.
- Keep it tiny, often 10 to 100 times smaller than kP.
- Too high, and it winds up (over-accumulates) and causes slow, lazy
  oscillation.
- Plenty of mechanisms are perfect with `kI = 0`. Start there.

`clockwork::PIDController` guards against wind-up for you. It caps the
accumulated sum (`integralCap`) and clears it whenever the error crosses zero, so
a big move can't leave a huge leftover push waiting to overshoot.

### The tuning recipe (do this in order)

1. Set `kI = 0` and `kD = 0`.
2. Raise **kP** until the mechanism reaches the target quickly and overshoots
   just a little. If it shakes hard, you went too far, so back off.
3. Raise **kD** until that overshoot and shake smooth out. If it starts buzzing,
   back off.
4. Only if it still stops slightly short of the target, add a **tiny kI** (start
   around 1/50th of kP) until it closes the gap. Otherwise leave `kI = 0`.

Golden rule: **P, then D, then maybe I.** Change one gain at a time and watch
what it does before you touch the next.

### Symptom, then which gain

| What you see | Likely cause | Do this |
|--------------|--------------|---------|
| Reaches target too slowly, or stops short | kP too low | Raise kP |
| Overshoots then oscillates | kP too high | Lower kP, or add kD |
| Fast approach but shakes at the end | Needs damping | Add or raise kD |
| Buzzes or jitters constantly | kD too high | Lower kD |
| Settles just short of target forever | Steady offset | Add a tiny kI |
| Slow lazy wobble that won't die | kI wind-up | Lower kI, or set it to 0 |

### One thing that matters: loop timing

A controller assumes it runs at a steady rhythm. Always put a fixed delay in your
loop (`pros::delay(10)` is typical) and keep it the same. If you change the loop
delay, your tuned gains change meaning and you have to re-tune. That is why every
example here ends its loop with a `pros::delay`.

---

## API reference

### `clockwork::Motion`

```cpp
clockwork::Motion motion(&chassis);
```

Every drive helper holds the heading it started at with a P controller
(`headingKp`) and bypasses the driver curve. The distance-based helpers measure
from the pose at the moment you call them, so give the chassis a valid pose first
with `setPose`. Pass negative speeds or distances to run in reverse.

| Method | Signature |
|--------|-----------|
| `driveFullThenSlow` | `(float fullDist, float slowDist, int slowSpeed, int timeoutMs, int fullSpeed = 127, float headingKp = 2.0f)` |
| `driveDistance`     | `(float dist, int maxSpeed = 127, int timeoutMs = 3000, float headingKp = 2.0f, float settleRange = 1.0f, float driveKp = 8.0f)` |
| `driveTimed`        | `(int ms, int speed, float headingKp = 2.0f)` |
| `turnBy`            | `(float degrees, int timeoutMs = 1500, int maxSpeed = 127)` |
| `driveUntilStalled` | `(int power, int timeoutMs = 3000, float headingKp = 2.0f)` returns `bool` |

- **driveFullThenSlow** runs full speed for `fullDist` inches, then `slowSpeed`
  for the next `slowDist`. Fast approach, soft arrival.
- **driveDistance** drives `dist` inches, easing off near the target, settling
  within `settleRange` inches or timing out. `driveKp` sets how hard it pushes
  per inch remaining (raise for a snappier approach, lower if it overshoots).
- **driveTimed** applies `speed` for `ms` milliseconds, then stops.
- **turnBy** turns `degrees` relative to your current heading (positive is
  clockwise) using the chassis's own turn PID. It blocks until it settles.
- **driveUntilStalled** drives at `power` until the robot stops moving (a wall,
  an obstacle) or times out. It returns `true` if it actually stalled. Great for
  squaring on a wall before an odom reset.

The heading correction is internally clamped, so a hard bump can't produce a
giant turn command that steals power from the throttle. Forward motion always
keeps priority. `headingKp` and `driveKp` are the kP gains explained
[above](#understanding-gains-kp-ki-kd). The defaults are sane starting points, so
tune them on your own robot.

### `clockwork::Roller`

```cpp
clockwork::Roller intake(&intake_motors, 127); // second arg is the default power
```

| Method | Description |
|--------|-------------|
| `in()`  | spin inward at the default power |
| `out()` | spin outward at the default power |
| `stop()` | command 0 |
| `spin(int power)` | explicit signed power, -127 to 127 |
| `pulse(int power, int ms)` | spin for `ms`, then stop (this one blocks) |
| `hold()` | set HOLD brake mode and brake in place |
| `stalled(double velThreshold = 5.0)` | `true` if velocity is under `velThreshold` RPM |
| `antiJam(int reversePower = 127, int reverseMs = 200, int jamHoldMs = 150, double velThreshold = 5.0)` | non-blocking auto-reverse on a jam |

`antiJam` is the easy way to keep an intake clear. Call it once per loop right
after you command the intake (`in`, `out`, or `spin`). It reads the last power
you sent, so it always knows which way you meant to run. If the intake is
commanded to move but its velocity stays under `velThreshold` for longer than
`jamHoldMs`, it reverses at `reversePower` for `reverseMs` to clear the jam, then
resumes the previous command, all without blocking your loop. It returns `true`
while a clearing burst is in progress, and does nothing while the intake is
stopped.

```cpp
while (true) {
    intake.in();
    intake.antiJam();      // clears jams and resumes intaking on its own
    pros::delay(20);
}
```

It works off motor velocity, not current, so a motor that free-spins without
intaking reads as "moving" and won't false-trigger. The flip side: a slipping jam
where the motor still spins won't be caught. Keep calling it every loop, or a
burst in progress never gets cancelled.

### `clockwork::PIDController`

```cpp
clockwork::PIDController pid(kP, kI, kD, integralCap = 0.0f, outputCap = 0.0f);
```

A standalone PID for anything LemLib doesn't drive: an arm, a lift, a flywheel, a
wall-align. See [Understanding gains](#understanding-gains-kp-ki-kd) for what the
three numbers mean and how to tune them.

| Method | Description |
|--------|-------------|
| `update(float error)` returns `float` | Call every loop with `target - measured`. Returns a motor power, clamped to `outputCap` if you set one |
| `reset()` | Forget history (the integral sum and last error). Call it at the start of a new move |
| `setGains(float kP, float kI, float kD)` | Swap gains at runtime, for example loaded versus empty |
| `settled(float tolerance, float stillness = 1.0f)` | `true` when the error is within `tolerance` and barely changing, so it has actually arrived rather than passing through at speed |

Two constructor extras: `integralCap` limits how much the integral can build up
(anti-windup; `0` disables it), and `outputCap` limits the returned power. Pass
`127` for `outputCap` so `update()` never hands a motor an out-of-range command.

```cpp
// arm: kP=0.9, kI=0, kD=4, integral cap 50, output clamped to motor range
clockwork::PIDController armPid(0.9f, 0.0f, 4.0f, 50.0f, 127.0f);

void moveArmTo(float targetDeg) {
    armPid.reset();
    while (!armPid.settled(2.0f)) {
        float measured = armSensor.get_position() / 100.0f;
        arm.move(armPid.update(targetDeg - measured));
        pros::delay(10); // steady loop timing, see the gains guide
    }
    arm.brake();
}
```

To hold a position forever, like keeping an arm up under gravity, skip the
`settled()` exit and just keep calling `update()` every loop.

### `clockwork::AutonSelector`

```cpp
clockwork::AutonSelector selector(&master); // pass your pros::Controller
```

A controller-driven autonomous selector. You register routines by name, the
driver scrolls them with the D-pad before the match, the current pick shows on
the controller and brain screens, and `run()` runs the choice in `autonomous()`.
It uses only core PROS, so it works on any project (no liblvgl or LLEMU).

| Method | Description |
|--------|-------------|
| `add(name, routine)` | Register a `void()` routine (a function or a lambda) under a display name |
| `poll()` | One iteration of the picker: reads the D-pad and redraws. Call it in a loop |
| `run()` | Run the selected routine. Call it from `autonomous()` |
| `next()` / `prev()` | Move the selection, wrapping around |
| `select(int)` / `select(name)` | Choose a specific routine (for example a default) |
| `draw()` | Push the current selection to both screens |
| `index()` / `count()` / `name()` | Read the current state |

```cpp
void initialize() {
    selector.add("Left rush",  left_rush);
    selector.add("Right safe", right_safe);
    selector.add("Do nothing", [] {});
    selector.draw();
}

void competition_initialize() {
    while (true) { selector.poll(); pros::delay(20); } // driver picks here
}

void autonomous() { selector.run(); }
```

RIGHT or DOWN on the D-pad moves to the next routine, LEFT or UP to the previous
one. It uses new-press detection, so holding the D-pad won't run off the end.

### `clockwork::SlewRateLimiter`

```cpp
clockwork::SlewRateLimiter slew(2.5f); // most it can move per call
```

Ramps a value toward a target by at most a fixed step each call, so the command
eases in instead of jumping. Use it to smooth driver throttle (so a hard shove
can't wheelie the robot or brown out the battery) or to bring a flywheel up to
speed gently.

| Method | Description |
|--------|-------------|
| `calculate(float target)` returns `float` | Step the output toward `target` and return the new value. Call once per loop |
| `reset(float value = 0)` | Snap straight to a value with no ramp |
| `value()` | The current output, without stepping it |

The step is per call, so it depends on your loop rate. At a 10 ms loop, a step of
about `2.5` ramps from 0 to full power (127) over roughly half a second. Keep the
loop delay steady and the ramp stays consistent.

```cpp
void opcontrol() {
    while (true) {
        int y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        left_mg.move(slew.calculate(y)); // eased instead of instant
        pros::delay(10);
    }
}
```

---

## Tuning cheat sheet

- **`headingKp`** (default `2.0`): power per degree of heading error during
  straight drives. If the robot drifts off heading, raise it. If it wags side to
  side, lower it.
- **`driveKp`** (default `8.0`, `driveDistance` only): power per inch of
  remaining distance. If it arrives too slowly, raise it. If it overshoots the
  target, lower it.
- **`settleRange`** (default `1.0` in): how close counts as "arrived." Tighten
  for precision, loosen if it hunts around the target.
- **`driveUntilStalled`** treats "stopped" as very little pose movement over a
  short window. That is fine on a normal drivetrain, but if your odom is noisy,
  give it a longer timeout.
- **`PIDController`**: follow the recipe in the gains guide, P then D then maybe I.

All the gains here are starting points, not tuned for any specific robot. Tune on
the field.

---

## Testing

The pure-logic classes (`PIDController` and `SlewRateLimiter`) have no PROS or
hardware dependency, so they come with tests you can run on your own computer, no
V5 brain needed:

```bash
make test        # or: bash test/run.sh
```

That builds `test/test_clockwork.cpp` with your system compiler and runs it. It
checks the proportional math, the output clamp, that a PID actually converges on
a target, and that the slew limiter ramps and never overshoots. Expected output:

```
CLOCKWORK host tests
15 checks, 0 failed
OK
```

The motion helpers and the selector talk to real hardware, so they are verified
by building the template into a project and running it on a robot.

---

## Building and releasing

You need the PROS toolchain (`pros` and `arm-none-eabi`) on your PATH.

```bash
pros make            # build the project + bin/clockwork.a
pros make template   # package clockwork@<version>.zip
```

To cut a release: bump `VERSION` in the `Makefile`, run `pros make template`,
attach the zip to a GitHub release, and add an entry to `depot.json`. The
template ships only the public headers (`include/clockwork/*.hpp`) and the
compiled archive (`firmware/clockwork.a`).

---

## Versioning and changelog

Semantic versioning. See **[CHANGELOG.md](CHANGELOG.md)**.

## Contributing

See **[CONTRIBUTING.md](CONTRIBUTING.md)**.

## License

MIT, see [LICENSE](LICENSE).
