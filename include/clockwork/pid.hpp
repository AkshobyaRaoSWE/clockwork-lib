#pragma once

namespace clockwork {

/**
 * @brief A tiny, self-contained PID controller for any subsystem you want to
 *        hold at a target: an arm, a lift, a flywheel, a wall-align, anything.
 *
 * LemLib already PID-controls the *drivetrain* for you. This class is for
 * everything else — the mechanisms lemlib does not touch. You give it the three
 * gains, then each loop you feed it the error (how far you are from where you
 * want to be) and it hands back a motor power.
 *
 * ### What the three gains mean (read this before you tune)
 *
 * The controller output is:
 *
 *     output = kP * error  +  kI * (running sum of error)  +  kD * (change in error)
 *
 * - **kP (proportional)** — the main dial. Output is `kP * error`, so the
 *   further you are from the target, the harder it pushes. Too low: the arm
 *   never quite gets there (sags short). Too high: it overshoots and shakes.
 *   Tune this FIRST, with kI and kD at 0, until it reaches the target and only
 *   slightly overshoots.
 * - **kD (derivative)** — the brake. It reacts to how fast the error is
 *   shrinking and pushes back against it, damping the overshoot and wobble that
 *   a high kP causes. Add this SECOND, a little at a time, until the shake from
 *   kP is gone. Too high: it gets jittery / buzzy.
 * - **kI (integral)** — the closer. It accumulates leftover error over time, so
 *   a small steady offset (e.g. gravity holding an arm just below target) keeps
 *   building output until it is finally corrected. Add this LAST, and keep it
 *   TINY (often 10-100x smaller than kP). Too high: it winds up and oscillates
 *   slowly. Many mechanisms need kI = 0.
 *
 * Tuning order, every time: **P, then D, then (maybe) I.**
 *
 * ### Units
 * `error` is in whatever you measure in (degrees, inches, RPM). `output` is in
 * whatever you drive with — for VEX motors that is -127..127, so set
 * `outputCap` to 127. The gains carry the conversion between the two.
 *
 * ### Loop timing
 * Call update() at a steady interval (e.g. `pros::delay(10)` each loop). The
 * math assumes a fixed time step and folds it into the gains, so if you change
 * the loop delay you will need to re-tune.
 */
class PIDController {
public:
    /**
     * @brief Build a controller from its three gains.
     *
     * @param kP          proportional gain (start here; tune first)
     * @param kI          integral gain (keep tiny; tune last; 0 is common)
     * @param kD          derivative gain (the damper; tune second)
     * @param integralCap absolute limit on the accumulated integral term, in
     *                    error units. Stops "integral windup" (the sum growing
     *                    huge during a long push and then overshooting wildly).
     *                    0 disables the cap. Default 0.
     * @param outputCap   absolute limit on the returned output. For a VEX motor
     *                    pass 127 so update() never returns an out-of-range
     *                    power. 0 disables the cap. Default 0.
     */
    PIDController(float kP, float kI, float kD, float integralCap = 0.0f,
                  float outputCap = 0.0f);

    /**
     * @brief Step the controller once and get a control output.
     *
     * Call this every loop with the current error (`target - measured`). The
     * sign of the return value follows the sign of the error, so a positive
     * error gives a positive output.
     *
     * @param error target minus current measurement (e.g. `targetDeg - armDeg`)
     * @return the control output, clamped to `outputCap` if one was set
     */
    float update(float error);

    /**
     * @brief Forget all history (integral sum and last error).
     *
     * Call this whenever you start a fresh move to a new target, so leftover
     * accumulation from the previous move does not kick the mechanism.
     */
    void reset();

    /// Swap in new gains at runtime (e.g. different gains loaded vs. empty).
    void setGains(float kP, float kI, float kD);

    /**
     * @brief A simple "have we arrived?" test for exiting a control loop.
     *
     * True when, on the most recent update(), the error is within @p tolerance
     * AND the error is barely changing (within @p stillness per step) — i.e.
     * the mechanism is both on target and no longer moving, not just passing
     * through the target at speed.
     *
     * @param tolerance error band counted as "at target" (error units)
     * @param stillness max per-update change in error still counted as settled
     *                  (error units; default 1.0)
     */
    bool settled(float tolerance, float stillness = 1.0f) const;

private:
    float m_kP, m_kI, m_kD;
    float m_integralCap;
    float m_outputCap;
    float m_integral;
    float m_prevError;
    float m_lastDelta;
    bool m_hasPrev;
};

} // namespace clockwork
