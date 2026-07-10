#pragma once

namespace clockwork {

// A trapezoidal motion profile: the smooth "speed up, cruise, slow down" shape
// you want for a move instead of slamming to full speed and back to zero.
//
// You give it how far to go, the top speed to allow, and how hard it may
// accelerate. It works out the timing and then tells you the target velocity and
// position at any moment. Feed those to a velocity controller (or just to your
// motors) each loop and the robot eases in and out on its own.
//
// If the distance is too short to ever reach the top speed, it does the right
// thing and gives you a triangle (speed up, then straight back down) instead of
// a trapezoid. Distance can be negative to profile a move in reverse; the
// velocity and position come back with the matching sign.
//
// Units are yours to pick, as long as they agree: if distance is in inches and
// velocity is inches/sec, then acceleration is inches/sec^2 and time is seconds.
class TrapezoidalProfile {
public:
    // distance: how far to travel (sign sets the direction).
    // maxVel:   the fastest it's allowed to go (a positive speed).
    // maxAccel: how quickly it may speed up and slow down (positive).
    TrapezoidalProfile(float distance, float maxVel, float maxAccel);

    // How long the whole move takes, in the same time unit as your rates.
    float totalTime() const;

    // The total distance this profile covers (what you asked for).
    float distance() const;

    // Target velocity at time t (t in the same unit as totalTime). It's 0 before
    // the start and 0 once the move is done.
    float velocityAt(float t) const;

    // Distance covered by time t, clamped to the full distance at the end.
    float positionAt(float t) const;

private:
    float m_distance;   // signed
    float m_maxVel;     // signed to match direction, magnitude actually reached
    float m_accel;      // positive
    float m_tAccel;     // time spent accelerating (= time decelerating)
    float m_tCruise;    // time spent at cruise (0 for a triangle profile)
    float m_total;      // total time
    float m_dir;        // +1 or -1
};

} // namespace clockwork
