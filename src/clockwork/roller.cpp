#include "clockwork/roller.hpp"
#include "pros/rtos.hpp"
#include <cstdlib>
#include <cmath>

namespace clockwork {

Roller::Roller(pros::MotorGroup* motors, int defaultPower)
    : m_motors(motors), m_power(std::abs(defaultPower)), m_lastCommand(0),
      m_clearing(false), m_jamSince(0), m_clearUntil(0) {}

void Roller::in() { spin(m_power); }

void Roller::out() { spin(-m_power); }

void Roller::stop() { spin(0); }

void Roller::spin(int power) {
	m_lastCommand = power;
	m_motors->move(power);
}

void Roller::hold() {
	m_lastCommand = 0;
	m_motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	m_motors->brake();
}

bool Roller::stalled(double velThreshold) const {
	return std::fabs(m_motors->get_actual_velocity()) < velThreshold;
}

void Roller::pulse(int power, int ms) {
	m_motors->move(power);
	pros::delay(ms);
	m_motors->move(0);
	m_lastCommand = 0;
}

bool Roller::antiJam(int reversePower, int reverseMs, int jamHoldMs,
                     double velThreshold) {
	// Idle intake: nothing to clear.
	if (m_lastCommand == 0) {
		m_clearing = false;
		m_jamSince = 0;
		return false;
	}

	const std::uint32_t now = pros::millis();

	// Mid-burst: keep reversing until the burst window closes, then resume.
	if (m_clearing) {
		if (now < m_clearUntil) {
			return true;
		}
		m_clearing = false;
		m_jamSince = 0;
		m_motors->move(m_lastCommand); // resume the commanded direction
		return false;
	}

	// Commanded to move but not moving -> time the stall.
	if (std::fabs(m_motors->get_actual_velocity()) < velThreshold) {
		if (m_jamSince == 0) {
			m_jamSince = now;
		} else if (now - m_jamSince > static_cast<std::uint32_t>(jamHoldMs)) {
			// Jam confirmed: kick back opposite the commanded direction.
			const int dir = (m_lastCommand > 0) ? -1 : 1;
			m_motors->move(dir * std::abs(reversePower));
			m_clearing = true;
			m_clearUntil = now + static_cast<std::uint32_t>(reverseMs);
			return true;
		}
	} else {
		m_jamSince = 0;
	}
	return false;
}

} // namespace clockwork
