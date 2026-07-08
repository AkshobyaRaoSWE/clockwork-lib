# Contributing to CLOCKWORK

Thanks for your interest. This is a small library, and the bar is simple, generic,
well-tested motion/intake helpers that sit on top of PROS + LemLib.

## Ground rules

- **Generic, not robot-specific.** `Motion` takes a `lemlib::Chassis*`, `Roller`
  takes a `pros::MotorGroup*`. No hard-coded ports, subsystems, or field
  constants.
- **Match the style.** Look at the existing files; keep the same brace and
  naming conventions. Public API is documented with Doxygen-style comments.
- **Every helper builds.** Run `pros make` and `pros make template` before
  opening a PR. If you can, verify against a real project via `pros c fetch` +
  `pros c apply`.

## Adding a helper

1. Declare it in the relevant header (`include/clockwork/*.hpp`) with a doc
   comment.
2. Implement it in the matching `src/clockwork/*.cpp`.
3. If it's a new file, add its header to `include/clockwork/clockwork.hpp`.
4. Add an entry to `CHANGELOG.md` under an unreleased/next-version heading.
5. Add a usage snippet to `EXAMPLES.md` if it's non-obvious.

## Releasing (maintainers)

1. Bump `VERSION` in the `Makefile`.
2. `pros make template` → `clockwork@<version>.zip`.
3. Create a GitHub release tagged `v<version>` with the zip attached.
4. Prepend the new version to `depot.json`.
5. Update `CHANGELOG.md`.
