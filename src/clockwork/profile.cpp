#include "clockwork/profile.hpp"
#include <cmath>

namespace clockwork {

TrapezoidalProfile::TrapezoidalProfile(float distance, float maxVel, float maxAccel)
    : m_distance(distance), m_maxVel(0.0f), m_accel(maxAccel),
      m_tAccel(0.0f), m_tCruise(0.0f), m_total(0.0f),
      m_dir(distance < 0 ? -1.0f : 1.0f) {
	const float D = std::fabs(distance);

	// Nothing to do (no distance) or nonsense limits: leave it a zero-length
	// profile so every query returns 0 instead of dividing by zero.
	if (D <= 0.0f || maxVel <= 0.0f || maxAccel <= 0.0f) {
		return;
	}

	// Distance used getting all the way up to maxVel (and the same coming down).
	const float accelDist = (maxVel * maxVel) / (2.0f * maxAccel);

	if (2.0f * accelDist <= D) {
		// Long enough to reach cruise: a real trapezoid.
		m_maxVel = maxVel;
		m_tAccel = maxVel / maxAccel;
		const float cruiseDist = D - 2.0f * accelDist;
		m_tCruise = cruiseDist / maxVel;
	} else {
		// Too short to reach maxVel: a triangle that peaks partway.
		m_maxVel = std::sqrt(D * maxAccel);
		m_tAccel = m_maxVel / maxAccel;
		m_tCruise = 0.0f;
	}
	m_total = 2.0f * m_tAccel + m_tCruise;
}

float TrapezoidalProfile::totalTime() const { return m_total; }

float TrapezoidalProfile::distance() const { return m_distance; }

float TrapezoidalProfile::velocityAt(float t) const {
	if (t <= 0.0f || t >= m_total) return 0.0f;

	float v;
	if (t < m_tAccel) {
		v = m_accel * t;                              // speeding up
	} else if (t < m_tAccel + m_tCruise) {
		v = m_maxVel;                                 // cruising
	} else {
		v = m_maxVel - m_accel * (t - m_tAccel - m_tCruise); // slowing down
	}
	if (v < 0.0f) v = 0.0f;
	if (v > m_maxVel) v = m_maxVel;
	return m_dir * v;
}

float TrapezoidalProfile::positionAt(float t) const {
	const float D = std::fabs(m_distance);
	if (t <= 0.0f) return 0.0f;
	if (t >= m_total) return m_distance; // arrived

	const float accelDist = 0.5f * m_accel * m_tAccel * m_tAccel;
	float p;
	if (t < m_tAccel) {
		p = 0.5f * m_accel * t * t;
	} else if (t < m_tAccel + m_tCruise) {
		p = accelDist + m_maxVel * (t - m_tAccel);
	} else {
		const float td = t - m_tAccel - m_tCruise;
		p = accelDist + m_maxVel * m_tCruise + (m_maxVel * td - 0.5f * m_accel * td * td);
	}
	if (p > D) p = D;
	return m_dir * p;
}

} // namespace clockwork
