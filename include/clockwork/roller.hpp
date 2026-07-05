#pragma once

#include "pros/motor_group.hpp"

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

    /// Spin at an explicit signed power, -127..127.
    void spin(int power);

    /**
     * @brief Spin at @p power for @p ms milliseconds, then stop. Blocking.
     *
     * Handy for autonomous score/eject actions where you want a fixed burst.
     */
    void pulse(int power, int ms);

private:
    pros::MotorGroup* m_motors;
    int m_power;
};

} // namespace clockwork
