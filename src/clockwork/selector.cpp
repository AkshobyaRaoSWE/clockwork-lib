#include "clockwork/selector.hpp"
#include "pros/screen.hpp"

namespace clockwork {

static const std::string kNone = "None";

AutonSelector::AutonSelector(pros::Controller* controller)
    : m_controller(controller), m_index(-1) {}

void AutonSelector::add(const std::string& name, std::function<void()> routine) {
	m_names.push_back(name);
	m_routines.push_back(std::move(routine));
	if (m_index < 0) m_index = 0;
}

void AutonSelector::next() {
	if (m_names.empty()) return;
	m_index = (m_index + 1) % static_cast<int>(m_names.size());
	draw();
}

void AutonSelector::prev() {
	if (m_names.empty()) return;
	const int n = static_cast<int>(m_names.size());
	m_index = (m_index - 1 + n) % n;
	draw();
}

void AutonSelector::select(int index) {
	if (m_names.empty()) { m_index = -1; return; }
	const int n = static_cast<int>(m_names.size());
	if (index < 0) index = 0;
	if (index >= n) index = n - 1;
	m_index = index;
	draw();
}

bool AutonSelector::select(const std::string& name) {
	for (int i = 0; i < static_cast<int>(m_names.size()); ++i) {
		if (m_names[i] == name) {
			m_index = i;
			draw();
			return true;
		}
	}
	return false;
}

int AutonSelector::index() const { return m_index; }

int AutonSelector::count() const { return static_cast<int>(m_names.size()); }

const std::string& AutonSelector::name() const {
	if (m_index < 0 || m_index >= static_cast<int>(m_names.size())) return kNone;
	return m_names[m_index];
}

void AutonSelector::run() {
	if (m_index >= 0 && m_index < static_cast<int>(m_routines.size())) {
		m_routines[m_index]();
	}
}

void AutonSelector::poll() {
	if (!m_controller) return;
	// get_digital_new_press only fires once per physical press, so holding the
	// D-pad won't run the selection off the end.
	if (m_controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) ||
	    m_controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		next();
	}
	if (m_controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) ||
	    m_controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
		prev();
	}
}

void AutonSelector::draw() {
	// Controller screen: one line, padded so it overwrites the old text.
	if (m_controller) {
		std::string label = name();
		if (label.size() > 18) label = label.substr(0, 18);
		label.resize(18, ' ');
		m_controller->set_text(1, 0, label.c_str());
	}
	// Brain screen.
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM, 1, "CLOCKWORK auton selector");
	pros::screen::print(pros::E_TEXT_MEDIUM, 3, "> %s", name().c_str());
	if (count() > 0) {
		pros::screen::print(pros::E_TEXT_MEDIUM, 4, "  %d / %d", m_index + 1, count());
	}
}

} // namespace clockwork
