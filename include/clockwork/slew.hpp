#pragma once

namespace clockwork {

// Limits how fast a value can change from one loop to the next (a slew-rate
// limiter). You feed it a target every loop and it walks its output toward that
// target by at most a fixed step per call, so the command ramps up instead of
// slamming. Two things I reach for it for:
//
//   - Smoothing the drive in opcontrol, so pinning the joystick can't wheelie
//     the robot or brown out the battery on a huge current spike.
//   - Easing a flywheel or lift up to speed instead of hitting it with full
//     power all at once.
//
// The step is per call, so how quick the ramp feels depends on how often you
// call it. At a 10 ms loop, going 0 to full power (127) over about half a second
// is ~50 calls, so a step of around 2.5 gets you there. It doesn't track real
// time on its own, so keep your loop delay steady and the ramp stays consistent.
class SlewRateLimiter {
public:
    // maxDeltaPerCall is the most the output can move toward the target in one
    // calculate() call. Bigger means a faster ramp.
    explicit SlewRateLimiter(float maxDeltaPerCall);

    // Step the output toward target by up to maxDeltaPerCall and return it. Once
    // it gets there it just returns the target. Call it once per loop with the
    // value you actually want (usually the joystick reading).
    float calculate(float target);

    // Jump straight to a value with no ramp (0 by default).
    void reset(float value = 0.0f);

    // The current output, without stepping it.
    float value() const;

private:
    float m_maxDelta;
    float m_current;
};

} // namespace clockwork
