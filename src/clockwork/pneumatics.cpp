#include "clockwork/pneumatics.hpp"

namespace clockwork {

Pneumatics::Pneumatics(std::uint8_t port, bool startExtended)
    : m_piston(port, startExtended), m_extended(startExtended) {}

void Pneumatics::extend() {
	m_piston.set_value(true);
	m_extended = true;
}

void Pneumatics::retract() {
	m_piston.set_value(false);
	m_extended = false;
}

void Pneumatics::set(bool out) { out ? extend() : retract(); }

void Pneumatics::toggle() { set(!m_extended); }

bool Pneumatics::extended() const { return m_extended; }

} // namespace clockwork
