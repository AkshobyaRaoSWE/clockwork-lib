#pragma once

namespace clockwork {

// Shapes a raw joystick reading so the middle of the stick is gentler than the
// ends. You get fine control for small nudges and still hit full power when you
// push it all the way. This is the trick that makes a drive feel precise instead
// of twitchy.
//
// - input:    the raw stick value, -127 to 127.
// - curve:    how much to bend it. 1.0 is a straight line (no shaping); bump it
//             up (2 or 3) for more finesse near the center.
// - deadband: anything this small (either direction) reads as 0, so stick drift
//             and a slightly-off resting position don't creep the robot.
//
// Returns the shaped value, still in -127 to 127, ready to hand to a motor.
float joystickCurve(int input, float curve = 2.0f, int deadband = 5);

} // namespace clockwork
