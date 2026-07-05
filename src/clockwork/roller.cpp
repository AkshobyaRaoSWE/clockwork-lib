#include "clockwork/roller.hpp"
#include "pros/rtos.hpp"
#include <cstdlib>
#include <cmath>

namespace clockwork {

Roller::Roller(pros::MotorGroup* motors, int defaultPower)
    : m_motors(motors), m_power(std::abs(defaultPower)) {}

void Roller::in() { m_motors->move(m_power); }

void Roller::out() { m_motors->move(-m_power); }

void Roller::stop() { m_motors->move(0); }

void Roller::spin(int power) { m_motors->move(power); }

void Roller::hold() {
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
}

} // namespace clockwork
