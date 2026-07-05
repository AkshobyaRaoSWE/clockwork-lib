#include "clockwork/motion.hpp"
#include "lemlib/util.hpp" // lemlib::angleError, degToRad
#include "pros/rtos.hpp"
#include <cstdint>
#include <cmath>
#include <cstdlib>

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

void Motion::driveDistance(float dist, int maxSpeed, int timeoutMs,
                           float headingKp, float settleRange) {
	const lemlib::Pose start = m_chassis->getPose();
	const float targetHeading = start.theta; // degrees
	const float hRad = lemlib::degToRad(targetHeading);
	// lemlib compass heading: 0 deg = +Y, increasing clockwise.
	// Forward unit vector = (sin, cos); project displacement onto it for signed travel.
	const float sinH = std::sin(hRad);
	const float cosH = std::cos(hRad);

	const float distKp = 8.0f; // power per inch of remaining distance
	const int minPower = 12;   // floor to overcome static friction near target
	const int cap = std::abs(maxSpeed);

	const std::uint32_t t0 = pros::millis();
	std::uint32_t settledSince = 0;

	while (pros::millis() - t0 < static_cast<std::uint32_t>(timeoutMs)) {
		const lemlib::Pose pose = m_chassis->getPose();
		const float traveled = (pose.x - start.x) * sinH + (pose.y - start.y) * cosH;
		const float err = dist - traveled;

		if (std::fabs(err) < settleRange) {
			if (settledSince == 0) {
				settledSince = pros::millis();
			}
			if (pros::millis() - settledSince > 150) {
				break;
			}
		} else {
			settledSince = 0;
		}

		int mag = static_cast<int>(std::fabs(distKp * err));
		if (mag > cap) {
			mag = cap;
		}
		if (mag < minPower && std::fabs(err) >= settleRange) {
			mag = minPower;
		}
		const int throttle = (err >= 0) ? mag : -mag;

		const float herr = lemlib::angleError(targetHeading, pose.theta, false);
		const int turn = static_cast<int>(headingKp * herr);

		m_chassis->arcade(throttle, turn, true);
		pros::delay(10);
	}

	m_chassis->arcade(0, 0, true);
}

void Motion::driveTimed(int ms, int speed, float headingKp) {
	const lemlib::Pose start = m_chassis->getPose();
	const float targetHeading = start.theta;
	const std::uint32_t t0 = pros::millis();

	while (pros::millis() - t0 < static_cast<std::uint32_t>(ms)) {
		const lemlib::Pose pose = m_chassis->getPose();
		const float herr = lemlib::angleError(targetHeading, pose.theta, false);
		const int turn = static_cast<int>(headingKp * herr);
		m_chassis->arcade(speed, turn, true);
		pros::delay(10);
	}

	m_chassis->arcade(0, 0, true);
}

} // namespace clockwork
