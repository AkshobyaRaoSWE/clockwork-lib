#include "clockwork/motion.hpp"
#include "lemlib/util.hpp" // lemlib::angleError
#include "pros/rtos.hpp"
#include <cstdint>

namespace clockwork {

Motion::Motion(lemlib::Chassis* chassis) : m_chassis(chassis) {}

void Motion::driveFullThenSlow(float fullDist, float slowDist, int slowSpeed,
                               int timeoutMs, int fullSpeed, float headingKp) {
	const lemlib::Pose start = m_chassis->getPose();
	const float targetHeading = start.theta; // degrees
	const float totalDist = fullDist + slowDist;
	const std::uint32_t t0 = pros::millis();

	while (pros::millis() - t0 < static_cast<std::uint32_t>(timeoutMs)) {
		const lemlib::Pose pose = m_chassis->getPose();
		const float traveled = start.distance(pose);
		if (traveled >= totalDist) {
			break;
		}

		const int throttle = (traveled < fullDist) ? fullSpeed : slowSpeed;
		// Hold the starting heading. angleError returns degrees in (-180, 180].
		const float err = lemlib::angleError(targetHeading, pose.theta, false);
		const int turn = static_cast<int>(headingKp * err);

		m_chassis->arcade(throttle, turn, true); // true -> skip driver curve
		pros::delay(10);
	}

	m_chassis->arcade(0, 0, true);
}

} // namespace clockwork
