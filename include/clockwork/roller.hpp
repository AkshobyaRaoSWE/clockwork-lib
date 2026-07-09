#pragma once

#include "pros/motor_group.hpp"
#include <cstdint>

namespace clockwork {

// A thin wrapper around a motor group for intakes and rollers. It turns the
// usual "motor1.move(x); motor2.move(x);" into readable in/out/stop/spin/pulse
// calls. It borrows the group pointer and never owns it, so you stay in charge
// of the motors.
class Roller {
public:
    // motors is your intake/roller group. defaultPower (0..127) is what in() and
    // out() use.
    explicit Roller(pros::MotorGroup* motors, int defaultPower = 127);

    void in();    // spin inward (intake) at the default power
    void out();   // spin outward (outtake) at the default power
    void stop();  // command 0
    void hold();  // set HOLD brake mode and actively brake in place

    // Spin at an explicit signed power, -127..127.
    void spin(int power);

    // True when the group's actual velocity is under velThreshold RPM. Use it to
    // roll your own jam handling, or just call antiJam() and let it do the work.
    bool stalled(double velThreshold = 5.0) const;

    // Spin at power for ms milliseconds, then stop. This one blocks, which is
    // what you want for a fixed score/eject burst in an autonomous routine.
    void pulse(int power, int ms);

    // Non-blocking anti-jam. Call it once per loop, right after you command the
    // intake (in/out/spin). If the intake is told to move but its velocity stays
    // under velThreshold for longer than jamHoldMs, it counts as jammed: the
    // group reverses at reversePower for reverseMs to clear it, then the previous
    // command picks back up on its own. It does nothing while the intake is
    // stopped, and since it never blocks it works in both opcontrol and auton.
    // Returns true while a clearing burst is running.
    bool antiJam(int reversePower = 127, int reverseMs = 200, int jamHoldMs = 150,
                 double velThreshold = 5.0);

private:
    pros::MotorGroup* m_motors;
    int m_power;
    int m_lastCommand;          // last power sent via in/out/spin/stop
    bool m_clearing;            // currently running an anti-jam burst
    std::uint32_t m_jamSince;   // when the current stall began (0 = none)
    std::uint32_t m_clearUntil; // millis timestamp the burst ends at
};

} // namespace clockwork
