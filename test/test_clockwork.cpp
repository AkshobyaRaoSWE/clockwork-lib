// Host-side tests for the parts of CLOCKWORK that are pure logic and need no
// robot: PIDController and SlewRateLimiter. Build and run them on your computer
// with test/run.sh (or `make test`). No PROS, no V5 brain required.

#include "clockwork/curve.hpp"
#include "clockwork/flywheel.hpp"
#include "clockwork/geometry.hpp"
#include "clockwork/pid.hpp"
#include "clockwork/profile.hpp"
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

// ---- stress / edge cases ---------------------------------------------------

static void test_pid_stress() {
	// Integral cap: with kP=0, output is only the capped integral term. Feed a
	// steady error and the sum should stop growing at the cap.
	clockwork::PIDController i(0.0f, 1.0f, 0.0f, 5.0f, 0.0f);
	for (int n = 0; n < 100; ++n) i.update(10.0f);
	CHECK(near(i.update(10.0f), 5.0f)); // integral clamped to +5
	// Same the other way.
	clockwork::PIDController i2(0.0f, 1.0f, 0.0f, 5.0f, 0.0f);
	for (int n = 0; n < 100; ++n) i2.update(-10.0f);
	CHECK(near(i2.update(-10.0f), -5.0f));

	// Converges from the other direction (negative target) too.
	clockwork::PIDController pid(0.5f, 0.0f, 0.1f, 0.0f, 127.0f);
	float measured = 0.0f;
	for (int n = 0; n < 1000; ++n) measured += 0.1f * pid.update(-80.0f - measured);
	CHECK(std::fabs(-80.0f - measured) < 1.0f);

	// A giant error with a big gain still respects the output cap.
	clockwork::PIDController hot(50.0f, 0.0f, 0.0f, 0.0f, 127.0f);
	CHECK(hot.update(100000.0f) <= 127.0f);
	CHECK(hot.update(-100000.0f) >= -127.0f);
}

static void test_slew_stress() {
	// A zero step means it never moves off its current value.
	clockwork::SlewRateLimiter stuck(0.0f);
	CHECK(near(stuck.calculate(127.0f), 0.0f));
	CHECK(near(stuck.calculate(127.0f), 0.0f));

	// A negative step is treated as its magnitude (no runaway).
	clockwork::SlewRateLimiter neg(-10.0f);
	CHECK(near(neg.calculate(100.0f), 10.0f));

	// It never overshoots, even when the target jumps around.
	clockwork::SlewRateLimiter s(10.0f);
	float last = 0.0f;
	const float targets[] = {100.0f, -100.0f, 50.0f, 0.0f, 127.0f};
	for (float tgt : targets) {
		for (int n = 0; n < 40; ++n) {
			float out = s.calculate(tgt);
			CHECK(std::fabs(out - last) <= 10.0f + 1e-3f); // step never exceeds max
			last = out;
		}
		CHECK(near(s.value(), tgt, 1e-3f)); // and it does reach each target
	}
}

static void test_curve_stress() {
	// Exactly at the deadband edge is still zero; one past it is not.
	CHECK(near(clockwork::joystickCurve(5, 2.0f, 5), 0.0f));
	CHECK(clockwork::joystickCurve(6, 2.0f, 5) > 0.0f);
	// A curve below 1 expands the middle (more speed for a small push).
	CHECK(clockwork::joystickCurve(64, 0.5f, 0) > clockwork::joystickCurve(64, 1.0f, 0));
	// A silly deadband is clamped, not a crash or divide-by-zero.
	CHECK(near(clockwork::joystickCurve(0, 2.0f, 999), 0.0f));
	CHECK(clockwork::joystickCurve(127, 2.0f, 999) > 0.0f);
	// Output magnitude never exceeds 127.
	for (int in = -127; in <= 127; ++in) {
		CHECK(std::fabs(clockwork::joystickCurve(in, 2.0f, 5)) <= 127.0f + 1e-3f);
	}
}

static void test_geometry() {
	// Facing +Y (heading 0): moving +Y is forward, +X is sideways (zero).
	CHECK(near(clockwork::signedForwardDistance(0, 0, 0, 5, 0), 5.0f, 1e-3f));
	CHECK(near(clockwork::signedForwardDistance(0, 0, 5, 0, 0), 0.0f, 1e-3f));
	// Facing +X (heading 90): now +X is forward.
	CHECK(near(clockwork::signedForwardDistance(0, 0, 5, 0, 90), 5.0f, 1e-3f));
	CHECK(near(clockwork::signedForwardDistance(0, 0, 0, 5, 90), 0.0f, 1e-3f));
	// Facing -Y (heading 180): moving +Y is now backward.
	CHECK(near(clockwork::signedForwardDistance(0, 0, 0, 5, 180), -5.0f, 1e-3f));
	// Reverse of the heading reads negative.
	CHECK(near(clockwork::signedForwardDistance(0, 0, 0, -3, 0), -3.0f, 1e-3f));
}

static void test_profile() {
	// A real trapezoid (long enough to cruise).
	clockwork::TrapezoidalProfile p(100.0f, 20.0f, 40.0f);
	CHECK(near(p.totalTime(), 5.5f, 1e-3f));
	CHECK(near(p.positionAt(p.totalTime()), 100.0f, 1e-2f)); // arrives exactly
	CHECK(near(p.positionAt(0.0f), 0.0f));
	CHECK(near(p.velocityAt(0.0f), 0.0f));
	CHECK(near(p.velocityAt(p.totalTime()), 0.0f));
	CHECK(near(p.velocityAt(2.5f), 20.0f, 1e-3f)); // cruising at max
	// Velocity never exceeds max and position never goes backward.
	float lastPos = -1.0f;
	for (float t = 0.0f; t <= p.totalTime() + 0.01f; t += 0.05f) {
		CHECK(std::fabs(p.velocityAt(t)) <= 20.0f + 1e-3f);
		CHECK(p.positionAt(t) >= lastPos - 1e-3f);
		lastPos = p.positionAt(t);
	}

	// Too short to reach max speed: a triangle that peaks below maxVel.
	clockwork::TrapezoidalProfile tri(4.0f, 100.0f, 50.0f);
	float peak = 0.0f;
	for (float t = 0.0f; t <= tri.totalTime(); t += 0.005f) {
		peak = std::fmax(peak, tri.velocityAt(t));
	}
	CHECK(peak < 100.0f);              // never got near maxVel
	CHECK(near(peak, std::sqrt(4.0f * 50.0f), 0.5f));
	CHECK(near(tri.positionAt(tri.totalTime()), 4.0f, 1e-2f));

	// Reverse move: everything comes back negative and still lands on target.
	clockwork::TrapezoidalProfile rev(-24.0f, 48.0f, 96.0f);
	CHECK(near(rev.positionAt(rev.totalTime()), -24.0f, 1e-2f));
	CHECK(rev.velocityAt(rev.totalTime() * 0.5f) < 0.0f);

	// Degenerate inputs don't blow up.
	clockwork::TrapezoidalProfile zero(0.0f, 48.0f, 96.0f);
	CHECK(near(zero.totalTime(), 0.0f));
	clockwork::TrapezoidalProfile bad(24.0f, 0.0f, 96.0f);
	CHECK(near(bad.totalTime(), 0.0f));
}

static void test_flywheel() {
	clockwork::FlywheelController fw(0.2117f, 0.05f, 127.0f);
	// Target 0 stops.
	CHECK(near(fw.update(0.0f, 300.0f), 0.0f));
	// Output stays inside [0, max] no matter the error.
	CHECK(fw.update(100.0f, 100000.0f) >= 0.0f); // way over speed -> clamped to 0
	CHECK(fw.update(600.0f, 0.0f) <= 127.0f);    // way under -> clamped to max

	// Converges on a first-order flywheel plant (steady state rpm = 4.72 * power).
	float rpm = 0.0f;
	for (int n = 0; n < 3000; ++n) {
		float power = fw.update(500.0f, rpm);
		rpm += 0.05f * (4.72f * power - rpm);
	}
	CHECK(std::fabs(500.0f - rpm) < 15.0f);
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
	test_pid_stress();
	test_slew_stress();
	test_curve_stress();
	test_geometry();
	test_profile();
	test_flywheel();

	std::printf("%d checks, %d failed\n", g_checks, g_fails);
	if (g_fails == 0) std::printf("OK\n");
	return g_fails == 0 ? 0 : 1;
}
