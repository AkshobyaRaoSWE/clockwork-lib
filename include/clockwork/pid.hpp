#pragma once

namespace clockwork {

// A tiny PID controller for anything you want to hold at a target: an arm, a
// lift, a flywheel, a wall-align, whatever. LemLib already PID-controls the
// drivetrain, so this is for the mechanisms it doesn't touch. You hand it the
// three gains, then every loop you give it the error (how far off you are) and
// it hands back a motor power.
//
// The output is:
//
//     output = kP * error  +  kI * (running sum of error)  +  kD * (change in error)
//
// The three gains, in the order you should tune them:
//
//   kP (proportional) is the main dial. Output is kP * error, so the further
//     off you are the harder it pushes. Too low and the arm sags short; too high
//     and it overshoots and shakes. Tune this first with kI and kD at 0.
//   kD (derivative) is the brake. It reacts to how fast the error is shrinking
//     and pushes back, damping the overshoot a high kP causes. Add it second, a
//     bit at a time, until the shake is gone. Too high and it buzzes.
//   kI (integral) is the closer. It adds up leftover error over time, so a small
//     steady offset (gravity holding an arm just below target) eventually gets
//     corrected. Add it last and keep it tiny (often 10-100x smaller than kP).
//     Plenty of mechanisms want kI = 0.
//
// So: P, then D, then maybe I. Error is in whatever you measure (degrees,
// inches, RPM) and output is in whatever you drive (for VEX motors that's
// -127..127, so pass outputCap = 127). The gains carry the conversion. Call
// update() on a steady loop; the math assumes a fixed time step, so if you
// change the loop delay you'll need to re-tune.
class PIDController {
public:
    // kP/kI/kD are the gains above. integralCap limits how big the running sum
    // can get, which stops "integral windup" (the sum ballooning during a long
    // push and then overshooting hard); 0 turns the cap off. outputCap limits
    // the returned power; pass 127 for a VEX motor so update() never returns
    // something out of range. 0 turns that off too.
    PIDController(float kP, float kI, float kD, float integralCap = 0.0f,
                  float outputCap = 0.0f);

    // Step the controller once. Call it every loop with (target - measured). The
    // sign of the return follows the sign of the error, and it's clamped to
    // outputCap if you set one.
    float update(float error);

    // Wipe the history (the running sum and last error). Do this when you start
    // a fresh move, so leftovers from the last one don't kick the mechanism.
    void reset();

    // Swap in new gains on the fly (say, different gains for loaded vs. empty).
    void setGains(float kP, float kI, float kD);

    // A simple "are we there yet?" check for ending a control loop. True when
    // the last update() was within tolerance of the target AND barely moving
    // (change under stillness), so it's actually settled instead of blowing
    // through the target at speed.
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
