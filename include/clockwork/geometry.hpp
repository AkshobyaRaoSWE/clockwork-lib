#pragma once

#include <cmath>

namespace clockwork {

// How far you've traveled along the direction you're facing, in inches, given
// where you started, where you are now, and the heading you're holding.
//
// This is the exact projection driveDistance() uses to know its progress. It
// follows lemlib's compass convention: heading 0 points along +Y and grows
// clockwise, so the forward unit vector is (sin, cos). Moving forward gives a
// positive result, backward gives a negative one, and pure sideways drift gives
// roughly zero, which is what lets a straight drive ignore small wobble.
inline float signedForwardDistance(float startX, float startY, float x, float y,
                                   float headingDeg) {
	const float h = headingDeg * static_cast<float>(M_PI) / 180.0f;
	return (x - startX) * std::sin(h) + (y - startY) * std::cos(h);
}

} // namespace clockwork
