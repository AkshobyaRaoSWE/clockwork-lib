#include "clockwork/slew.hpp"
#include <cmath>

namespace clockwork {

SlewRateLimiter::SlewRateLimiter(float maxDeltaPerCall)
    : m_maxDelta(std::fabs(maxDeltaPerCall)), m_current(0.0f) {}

float SlewRateLimiter::calculate(float target) {
	float delta = target - m_current;
	if (delta > m_maxDelta) delta = m_maxDelta;
	else if (delta < -m_maxDelta) delta = -m_maxDelta;
	m_current += delta;
	return m_current;
}

void SlewRateLimiter::reset(float value) { m_current = value; }

float SlewRateLimiter::value() const { return m_current; }

} // namespace clockwork
