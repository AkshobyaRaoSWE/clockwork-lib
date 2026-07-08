#include "clockwork/pid.hpp"
#include <cmath>

namespace clockwork {

PIDController::PIDController(float kP, float kI, float kD, float integralCap,
                            float outputCap)
    : m_kP(kP), m_kI(kI), m_kD(kD), m_integralCap(integralCap),
      m_outputCap(outputCap), m_integral(0.0f), m_prevError(0.0f),
      m_lastDelta(0.0f), m_hasPrev(false) {}

float PIDController::update(float error) {
	// Anti-windup: when the error crosses zero (we blew past the target), the
	// accumulated push is now working against us, so drop it.
	if (m_hasPrev && ((error > 0) != (m_prevError > 0))) {
		m_integral = 0.0f;
	}

	m_integral += error;
	if (m_integralCap > 0.0f) {
		if (m_integral > m_integralCap) m_integral = m_integralCap;
		if (m_integral < -m_integralCap) m_integral = -m_integralCap;
	}

	const float derivative = m_hasPrev ? (error - m_prevError) : 0.0f;
	m_lastDelta = derivative;
	m_prevError = error;
	m_hasPrev = true;

	float output = m_kP * error + m_kI * m_integral + m_kD * derivative;
	if (m_outputCap > 0.0f) {
		if (output > m_outputCap) output = m_outputCap;
		if (output < -m_outputCap) output = -m_outputCap;
	}
	return output;
}

void PIDController::reset() {
	m_integral = 0.0f;
	m_prevError = 0.0f;
	m_lastDelta = 0.0f;
	m_hasPrev = false;
}

void PIDController::setGains(float kP, float kI, float kD) {
	m_kP = kP;
	m_kI = kI;
	m_kD = kD;
}

bool PIDController::settled(float tolerance, float stillness) const {
	return m_hasPrev && std::fabs(m_prevError) <= tolerance &&
	       std::fabs(m_lastDelta) <= stillness;
}

} // namespace clockwork
