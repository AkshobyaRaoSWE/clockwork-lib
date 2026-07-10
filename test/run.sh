#!/usr/bin/env bash
# Build and run the CLOCKWORK host tests with your system compiler.
# These cover the pure-logic classes (PIDController, SlewRateLimiter) and need
# no PROS install and no V5 brain. Run from anywhere:  test/run.sh
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
out="$root/test/run"

CXX="${CXX:-c++}"
"$CXX" -std=c++17 -Wall -Wextra \
  -I "$root/include" \
  "$root/test/test_clockwork.cpp" \
  "$root/src/clockwork/pid.cpp" \
  "$root/src/clockwork/slew.cpp" \
  "$root/src/clockwork/curve.cpp" \
  "$root/src/clockwork/profile.cpp" \
  "$root/src/clockwork/flywheel.cpp" \
  -o "$out"

"$out"
