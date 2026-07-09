// Host-side tests for the parts of CLOCKWORK that are pure logic and need no
// robot: PIDController and SlewRateLimiter. Build and run them on your computer
// with test/run.sh (or `make test`). No PROS, no V5 brain required.

#include "clockwork/curve.hpp"
#include "clockwork/pid.hpp"
#include "clockwork/slew.hpp"
#include "clockwork/toggle.hpp"
#include <cmath>
#include <cstdio>

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond)                                                            \
	do {                                                                       \
		g_checks++;                                                            \
		if (!(cond)) {                                                         \
			g_fails++;                                                         \
			std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
		}                                                                      \
	} while (0)

static bool near(float a, float b, float eps = 1e-4f) {
	return std::fabs(a - b) <= eps;
}

static void test_pid_proportional() {
	// Pure P: output is exactly kP * error.
	clockwork::PIDController pid(2.0f, 0.0f, 0.0f);
	CHECK(near(pid.update(10.0f), 20.0f));
	pid.reset();
	CHECK(near(pid.update(-4.0f), -8.0f)); // sign follows the error
}

static void test_pid_output_cap() {
	// A huge kP with a cap must never exceed the cap, either direction.
	clockwork::PIDController pid(100.0f, 0.0f, 0.0f, 0.0f, 127.0f);
	CHECK(near(pid.update(10.0f), 127.0f));
	pid.reset();
	CHECK(near(pid.update(-10.0f), -127.0f));
}

static void test_pid_converges() {
	// Drive a simple plant (an integrator) to a target and confirm it arrives.
	clockwork::PIDController pid(0.5f, 0.0f, 0.1f, 0.0f, 127.0f);
	const float target = 100.0f;
	float measured = 0.0f;
	for (int i = 0; i < 1000; ++i) {
		float out = pid.update(target - measured);
		measured += 0.1f * out; // plant responds to the command
	}
	CHECK(std::fabs(target - measured) < 1.0f); // basically arrived
	CHECK(pid.settled(1.0f));                   // and reports settled
}

static void test_pid_settled_after_reset() {
	// settled() needs history; a fresh/reset controller is never "settled".
	clockwork::PIDController pid(1.0f, 0.0f, 0.0f);
	pid.update(0.0f);           // tiny error, one sample
	pid.reset();
	CHECK(!pid.settled(1.0f));  // no history after reset
}

static void test_slew_ramp_up() {
	// Steps toward the target by at most the max delta, then holds.
	clockwork::SlewRateLimiter slew(10.0f);
	CHECK(near(slew.calculate(100.0f), 10.0f));
	CHECK(near(slew.calculate(100.0f), 20.0f));
	CHECK(near(slew.calculate(100.0f), 30.0f));
	for (int i = 0; i < 50; ++i) slew.calculate(100.0f);
	CHECK(near(slew.value(), 100.0f)); // reaches and holds, never overshoots
}

static void test_slew_ramp_down_and_reset() {
	clockwork::SlewRateLimiter slew(10.0f);
	slew.reset(50.0f);
	CHECK(near(slew.value(), 50.0f));
	CHECK(near(slew.calculate(0.0f), 40.0f));
	CHECK(near(slew.calculate(0.0f), 30.0f));
	// When the target is within one step, it snaps exactly to it.
	slew.reset(3.0f);
	CHECK(near(slew.calculate(0.0f), 0.0f));
}

static void test_toggle() {
	clockwork::Toggle t;
	CHECK(t.state() == false);          // starts off
	CHECK(t.update(false) == false);    // no press, no change
	CHECK(t.update(true) == true);      // rising edge flips it on
	CHECK(t.update(true) == true);      // still held, stays on
	CHECK(t.update(false) == true);     // release, stays on
	CHECK(t.update(true) == false);     // next press flips it off
	t.set(true);
	CHECK(t.state() == true);           // set() overrides
	clockwork::Toggle t2(true);
	CHECK(t2.state() == true);          // honors the initial value
}

static void test_joystick_curve() {
	// Deadband zeroes out small input.
	CHECK(near(clockwork::joystickCurve(0, 1.0f, 5), 0.0f));
	CHECK(near(clockwork::joystickCurve(4, 1.0f, 5), 0.0f));
	// Linear curve with no deadband passes the extremes straight through.
	CHECK(near(clockwork::joystickCurve(127, 1.0f, 0), 127.0f));
	CHECK(near(clockwork::joystickCurve(-127, 1.0f, 0), -127.0f));
	// A higher curve pulls the middle down (finer low-speed control).
	float linearMid = clockwork::joystickCurve(64, 1.0f, 0);
	float curvedMid = clockwork::joystickCurve(64, 2.0f, 0);
	CHECK(curvedMid < linearMid);
	// Sign is always preserved.
	CHECK(clockwork::joystickCurve(-64, 2.0f, 0) < 0.0f);
}

int main() {
	std::printf("CLOCKWORK host tests\n");
	test_pid_proportional();
	test_pid_output_cap();
	test_pid_converges();
	test_pid_settled_after_reset();
	test_slew_ramp_up();
	test_slew_ramp_down_and_reset();
	test_toggle();
	test_joystick_curve();

	std::printf("%d checks, %d failed\n", g_checks, g_fails);
	if (g_fails == 0) std::printf("OK\n");
	return g_fails == 0 ? 0 : 1;
}
