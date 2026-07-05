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

    /**
     * @brief Drive straight a relative distance, holding the current heading.
     *
     * Drives @p dist inches from the current pose along the current heading,
     * slowing proportionally as it approaches the target so it arrives cleanly.
     * Positive @p dist drives forward, negative drives in reverse. Heading is
     * held with a P controller and the driver curve is bypassed. Blocks until
     * the robot settles within @p settleRange of the target or @p timeoutMs
     * elapses, then stops.
     *
     * @param dist        signed distance in inches (+ forward, - reverse)
     * @param maxSpeed    power cap, 0..127 (default 127)
     * @param timeoutMs   safety cap; the motion always ends by this time
     * @param headingKp   heading P gain, power per degree of error (default 2.0)
     * @param settleRange inches from target counted as "arrived" (default 1.0)
     */
    void driveDistance(float dist, int maxSpeed = 127, int timeoutMs = 3000,
                       float headingKp = 2.0f, float settleRange = 1.0f);

    /**
     * @brief Drive straight at a fixed power for a fixed time, holding heading.
     *
     * Open-loop timed drive: applies @p speed for @p ms milliseconds while
     * holding the starting heading with a P controller, then stops. Useful for
     * ramming into a wall or field element to square up.
     *
     * @param ms         duration in milliseconds
     * @param speed      drive power, -127..127
     * @param headingKp  heading P gain, power per degree of error (default 2.0)
     */
    void driveTimed(int ms, int speed, float headingKp = 2.0f);

private:
    lemlib::Chassis* m_chassis;
};

} // namespace clockwork
