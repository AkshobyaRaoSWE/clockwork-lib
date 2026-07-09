#include "clockwork/curve.hpp"
#include <cmath>
#include <cstdlib>

namespace clockwork {

float joystickCurve(int input, float curve, int deadband) {
	// Keep the deadband sane so we never divide by zero below.
	if (deadband < 0) deadband = 0;
	if (deadband > 126) deadband = 126;

	const int mag = std::abs(input);
	if (mag <= deadband) return 0.0f;

	const float sign = input < 0 ? -1.0f : 1.0f;
	// How far past the deadband we are, as 0..1, so the output picks up smoothly
	// from zero right at the edge of the deadband instead of jumping.
	float x = static_cast<float>(mag - deadband) / static_cast<float>(127 - deadband);
	if (x > 1.0f) x = 1.0f;

	return sign * std::pow(x, curve) * 127.0f;
}

} // namespace clockwork
