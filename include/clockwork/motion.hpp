#pragma once

#include "lemlib/chassis/chassis.hpp"

namespace clockwork {

// Reusable motion primitives on top of a lemlib::Chassis. Build one with a
// pointer to an already-configured chassis and call these from your autonomous
// routines. It borrows the chassis, never owns it.
//
// Every drive here holds the heading it started at with a simple P controller
// (headingKp) and skips the driver curve. The distance-based ones measure from
// wherever the robot is when you call them, so give the chassis a real pose
// first with setPose. Negative speeds or distances run things in reverse.
class Motion {
public:
    // chassis must already be configured and calibrated.
    explicit Motion(lemlib::Chassis* chassis);

    // Drive straight in two phases: fullSpeed for the first fullDist inches, then
    // ease down to slowSpeed for the next slowDist. Fast approach, soft arrival.
    // Blocks until it covers the whole distance or timeoutMs runs out, then
    // stops. slowSpeed and fullSpeed are -127..127, headingKp is power per degree
    // of heading error.
    void driveFullThenSlow(float fullDist, float slowDist, int slowSpeed,
                           int timeoutMs, int fullSpeed = 127,
                           float headingKp = 2.0f);

    // Drive dist inches from where you are, easing off as you near the target so
    // you arrive clean. Positive is forward, negative is reverse. Blocks until it
    // settles within settleRange inches of the target or times out. maxSpeed caps
    // the power (0..127); driveKp is how hard it pushes per inch remaining (raise
    // for a snappier approach, lower if it overshoots).
    void driveDistance(float dist, int maxSpeed = 127, int timeoutMs = 3000,
                       float headingKp = 2.0f, float settleRange = 1.0f,
                       float driveKp = 8.0f);

    // Open-loop: apply speed (-127..127) for ms milliseconds while holding the
    // starting heading, then stop. Good for ramming a wall or field element to
    // square up.
    void driveTimed(int ms, int speed, float headingKp = 2.0f);

    // Turn degrees relative to the current heading (positive clockwise) using the
    // chassis's own tuned turn PID. Blocks until it settles or times out.
    void turnBy(float degrees, int timeoutMs = 1500, int maxSpeed = 127);

    // Drive at power and hold heading until the robot stops moving (it hit a wall
    // or field element) or times out. The stall is read off the odometry pose, so
    // there's no motor-current wiring to set up. Returns true if it actually
    // stalled, false if it timed out. Great for squaring on a wall before you
    // reset the pose.
    bool driveUntilStalled(int power, int timeoutMs = 3000,
                           float headingKp = 2.0f);

private:
    lemlib::Chassis* m_chassis;
};

} // namespace clockwork
