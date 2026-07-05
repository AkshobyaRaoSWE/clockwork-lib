# CLOCKWORK

A PROS library of extra motion primitives for the VEX V5, built on top of
[LemLib](https://lemlib.readthedocs.io/). Install it as a PROS template and call
its primitives from your autonomous routines — no need to copy source files
between projects.

Created for V5RC team 2360C.

---

## What it does

CLOCKWORK layers reusable motion helpers on top of a configured
`lemlib::Chassis`. The flagship primitive is a **two-phase straight drive**:
full speed for a first distance, then a controlled decelerate to a slower speed
for a second distance — fast approach, soft arrival — with a P heading hold so
the robot tracks straight.

---

## Installation

CLOCKWORK requires an existing PROS project that already has **LemLib**
installed (`pros c install LemLib`).

### From the release zip

```bash
# grab clockwork@x.y.z.zip from the Releases page, then:
pros c fetch clockwork@1.0.0.zip     # import the template
pros c apply clockwork               # install it into the current project
```

### From the depot

```bash
pros c add-depot clockwork https://raw.githubusercontent.com/AkshobyaRaoSWE/clockwork-lib/main/depot.json
pros c apply clockwork
```

> Note: `pros c apply <path-to-zip>` does **not** read a filesystem path — use
> `pros c fetch <zip>` first, then `pros c apply clockwork`.

---

## Usage

```cpp
#include "clockwork/clockwork.hpp"

// `chassis` is your configured, calibrated lemlib::Chassis
void autonomous() {
    clockwork::Motion motion(&chassis);

    chassis.setPose(0, 0, 0);
    // Full speed for 24 in, then decelerate to 40 power for the next 12 in.
    motion.driveFullThenSlow(24, 12, 40, 3000);
}
```

---

## API

### `clockwork::Motion`

Construct with a pointer to a configured chassis; it borrows the pointer and
never owns it.

```cpp
explicit Motion(lemlib::Chassis* chassis);
```

#### `driveFullThenSlow`

```cpp
void driveFullThenSlow(float fullDist, float slowDist, int slowSpeed,
                       int timeoutMs, int fullSpeed = 127,
                       float headingKp = 2.0f);
```

Drives straight with a two-phase speed profile: `fullSpeed` for the first
`fullDist` inches, then `slowSpeed` for the next `slowDist` inches. Holds the
starting heading with a P controller and bypasses the driver curve. Blocks until
the total distance is covered or `timeoutMs` elapses, then stops.

| Param       | Meaning                                                    |
|-------------|------------------------------------------------------------|
| `fullDist`  | inches driven at `fullSpeed`                               |
| `slowDist`  | inches driven at `slowSpeed` after the full-speed phase    |
| `slowSpeed` | reduced power, −127..127 (the "slow" speed)               |
| `timeoutMs` | safety cap; the motion always ends by this time            |
| `fullSpeed` | power for the first phase, −127..127 (default 127)        |
| `headingKp` | P gain, power per degree of heading error (default 2.0)    |

Distance is measured from the pose at the call site, so the chassis must already
have a valid pose (call `setPose` first). Pass negative speeds to run the
profile in reverse.

---

## Building the library

Requires the PROS toolchain (`pros` + `arm-none-eabi` on your `PATH`).

```bash
pros make            # build the project + bin/clockwork.a
pros make template   # package clockwork@<version>.zip
```

The template packages the public headers (`include/clockwork/*.hpp`) and the
compiled archive (`firmware/clockwork.a`). Bump `VERSION` in the `Makefile`
before cutting a release.

---

## License

MIT — see [LICENSE](LICENSE).
