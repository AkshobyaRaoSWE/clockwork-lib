#pragma once

#include "lemlib/chassis/chassis.hpp"

namespace clockwork {

/**
 * @brief Reusable motion primitives layered on top of a lemlib::Chassis.
 *
 * Construct a Motion with a pointer to an already-configured chassis, then call
 * its primitives from an autonomous routine. Motion never owns the chassis; it
 * only borrows the pointer.
 */
class Motion {
public:
    /**
     * @brief Create a Motion helper.
     * @param chassis pointer to a configured, calibrated lemlib chassis
     */
    explicit Motion(lemlib::Chassis* chassis);

    /**
     * @brief Drive straight with a two-phase speed profile.
     *
     * Drives at @p fullSpeed for the first @p fullDist inches, then decelerates
     * to @p slowSpeed for the next @p slowDist inches. Heading is held with a
     * simple P controller so the robot tracks straight, and the driver curve is
     * bypassed. Blocks until the total distance is covered or @p timeoutMs
     * elapses, then stops the drive.
     *
     * Distance is measured from the pose at the call site, so the chassis must
     * already have a valid pose (call setPose first). Pass negative speeds to
     * run the profile in reverse.
     *
     * @param fullDist   inches driven at @p fullSpeed
     * @param slowDist   inches driven at @p slowSpeed after the full phase
     * @param slowSpeed  reduced power, -127..127 (the "slow" speed)
     * @param timeoutMs  safety cap; the motion always ends by this time
     * @param fullSpeed  power for the first phase, -127..127 (default 127)
     * @param headingKp  P gain, power per degree of heading error (default 2.0)
     */
    void driveFullThenSlow(float fullDist, float slowDist, int slowSpeed,
                           int timeoutMs, int fullSpeed = 127,
                           float headingKp = 2.0f);

private:
    lemlib::Chassis* m_chassis;
};

} // namespace clockwork
