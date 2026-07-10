#pragma once

namespace clockwork {

// A velocity controller for a flywheel (or any wheel you want spinning at a set
// speed). A plain position PID is the wrong tool here: a flywheel is never
// "there", it just needs to hold an RPM while shots knock it down. So this uses
// the combo that actually works for spinning masses:
//
//   output = kFF * targetRpm  +  kP * (targetRpm - measuredRpm)
//
// The feedforward term (kFF) does the heavy lifting: it guesses the power needed
// to hold the target speed, so the wheel gets right up to it. The proportional
// term (kP) trims the leftover error and helps it recover after a shot drags it
// down. Output is clamped to [0, outputMax] because a flywheel only spins one
// way.
//
// Tuning: set kP to 0 and raise kFF until the wheel settles close to the target
// on its own (kFF is roughly outputMax / maxRpm as a starting point). Then add a
// little kP so it snaps back fast after a shot.
class FlywheelController {
public:
    // kFF: feedforward, output per unit of target RPM.
    // kP:  proportional trim, output per unit of RPM error.
    // outputMax: power ceiling (127 for a VEX motor).
    FlywheelController(float kFF, float kP, float outputMax = 127.0f);

    // Call every loop with the target and the wheel's measured RPM. Returns a
    // motor power in [0, outputMax].
    float update(float targetRpm, float measuredRpm);

    // Swap the gains at runtime (e.g. a different speed preset).
    void setGains(float kFF, float kP);

private:
    float m_kFF;
    float m_kP;
    float m_max;
};

} // namespace clockwork
