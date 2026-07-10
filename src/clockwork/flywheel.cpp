#include "clockwork/flywheel.hpp"

namespace clockwork {

FlywheelController::FlywheelController(float kFF, float kP, float outputMax)
    : m_kFF(kFF), m_kP(kP), m_max(outputMax) {}

float FlywheelController::update(float targetRpm, float measuredRpm) {
	// Target 0 means stop, no feedforward push.
	if (targetRpm <= 0.0f) return 0.0f;

	float out = m_kFF * targetRpm + m_kP * (targetRpm - measuredRpm);
	if (out < 0.0f) out = 0.0f;   // a flywheel only spins one way
	if (out > m_max) out = m_max;
	return out;
}

void FlywheelController::setGains(float kFF, float kP) {
	m_kFF = kFF;
	m_kP = kP;
}

} // namespace clockwork
