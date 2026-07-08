#pragma once

#include "pros/motor_group.hpp"
#include <cstdint>

namespace clockwork {

/**
 * @brief Thin wrapper around a motor group for intakes and rollers.
 *
 * Replaces the usual pair of `motor1.move(x); motor2.move(x);` calls with
 * readable in/out/stop/spin/pulse commands. Borrows the group pointer; never
 * owns it.
 */
class Roller {
public:
    /**
     * @brief Wrap a motor group.
     * @param motors       pointer to the intake/roller motor group
     * @param defaultPower power used by in()/out(), 0..127 (default 127)
     */
    explicit Roller(pros::MotorGroup* motors, int defaultPower = 127);

    /// Spin inward (intake) at the default power.
    void in();

    /// Spin outward (outtake) at the default power.
    void out();

    /// Stop the motors (coast to 0 command).
    void stop();

    /// Actively hold position (sets HOLD brake mode and brakes).
    void hold();

    /**
     * @brief Whether the motors are stalled (velocity below a threshold).
     *
     * Reports true when the group's actual velocity is under @p velThreshold
     * RPM. Use it to build your own anti-jam logic, or just reach for antiJam().
     *
     * @param velThreshold RPM below which the motors count as stalled (default 5)
     */
    bool stalled(double velThreshold = 5.0) const;

    /// Spin at an explicit signed power, -127..127.
    void spin(int power);

    /**
     * @brief Spin at @p power for @p ms milliseconds, then stop. Blocking.
     *
     * Handy for autonomous score/eject actions where you want a fixed burst.
     */
    void pulse(int power, int ms);

    /**
     * @brief Non-blocking anti-jam: auto-reverse a burst when the intake jams.
     *
     * Call this once per loop, right after commanding the intake (`in()`,
     * `out()`, or `spin()`). While the intake is commanded to move but its
     * velocity stays under @p velThreshold for longer than @p jamHoldMs, it is
     * treated as jammed: the group is reversed at @p reversePower for
     * @p reverseMs to clear the jam, then the previous command resumes
     * automatically. Does nothing while the intake is stopped.
     *
     * Because it never blocks, it works in both opcontrol and autonomous. The
     * intake keeps clearing jams on its own while the rest of your loop runs.
     *
     * @param reversePower  power for the clearing burst, 0..127 (default 127)
     * @param reverseMs     how long each clearing burst lasts (default 200 ms)
     * @param jamHoldMs     stalled time before a jam is declared (default 150 ms)
     * @param velThreshold  RPM below which the intake counts as stalled (default 5)
     * @return true while a clearing burst is in progress, false otherwise
     */
    bool antiJam(int reversePower = 127, int reverseMs = 200, int jamHoldMs = 150,
                 double velThreshold = 5.0);

private:
    pros::MotorGroup* m_motors;
    int m_power;
    int m_lastCommand;               // last power sent via in/out/spin/stop
    bool m_clearing;                 // currently running an anti-jam burst
    std::uint32_t m_jamSince;        // when the current stall began (0 = none)
    std::uint32_t m_clearUntil;      // millis timestamp the burst ends at
};

} // namespace clockwork
