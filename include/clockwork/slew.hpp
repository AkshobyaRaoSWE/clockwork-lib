#pragma once

namespace clockwork {

/**
 * @brief Limits how fast a value is allowed to change from one loop to the next.
 *
 * Also called a slew-rate limiter. You feed it a target every loop and it walks
 * its output toward that target by at most a fixed step per call, so the command
 * ramps instead of jumping. Two common uses on a VEX robot:
 *
 * - Smooth the drive in opcontrol so slamming the joystick can't wheelie the
 *   robot or brown out the battery from a huge current spike.
 * - Ease a flywheel or lift up to speed instead of hitting it with full power.
 *
 * The step is "per call", so it depends on how often you call it. If your loop
 * runs every 10 ms and you want to go from 0 to full power (127) over about half
 * a second, that is ~50 calls, so a step of about 2.5 gets you there.
 *
 * It has no idea about time on its own; keep your loop delay steady (the same
 * fixed `pros::delay`) and the ramp stays consistent.
 */
class SlewRateLimiter {
public:
    /**
     * @brief Create a limiter.
     * @param maxDeltaPerCall the most the output may move toward the target on a
     *                        single calculate() call. Bigger means a faster ramp.
     */
    explicit SlewRateLimiter(float maxDeltaPerCall);

    /**
     * @brief Step the output toward @p target and return the new value.
     *
     * Moves the internal value toward @p target by at most `maxDeltaPerCall`.
     * Once it reaches the target it just returns the target. Call it once per
     * loop with the value you actually want (e.g. the joystick reading).
     */
    float calculate(float target);

    /// Snap the internal value straight to @p value with no ramp (default 0).
    void reset(float value = 0.0f);

    /// The current output value, without stepping it.
    float value() const;

private:
    float m_maxDelta;
    float m_current;
};

} // namespace clockwork
