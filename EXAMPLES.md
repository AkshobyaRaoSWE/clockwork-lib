# Examples

Full routines using the CLOCKWORK library. In each, `chassis` is a configured
`lemlib::Chassis` and `intake` is a `clockwork::Roller` wrapping your intake
motor group, both declared as globals in your own `robot.cpp`.

```cpp
// robot.cpp
pros::MotorGroup intake_motors({-12, -19});
clockwork::Roller intake(&intake_motors);
// ... chassis, sensors, etc.
```

```cpp
// robot.hpp
extern pros::MotorGroup intake_motors;
extern clockwork::Roller intake;
extern lemlib::Chassis chassis;
```

---

## 1. Straight rush + score

Intake on, sprint to the goal fast but arrive gently, then eject.

```cpp
#include "clockwork/clockwork.hpp"

void rush_and_score() {
    clockwork::Motion motion(&chassis);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(0, 0, 0);

    intake.in();
    motion.driveFullThenSlow(36, 8, 45, 2500); // 36 in full, 8 in easing to 45
    intake.pulse(-127, 400);                   // eject
}
```

## 2. Wall-align and reset odometry

Ram a wall, confirm contact, and re-zero the pose to kill accumulated drift.

```cpp
void align_to_wall() {
    clockwork::Motion motion(&chassis);
    chassis.setPose(0, 0, 0);

    if (motion.driveUntilStalled(70, 1500)) {
        // squared against the wall, reset the known axis
        lemlib::Pose p = chassis.getPose();
        chassis.setPose(p.x, 0, 0);
    }
}
```

## 3. Box pattern with relative turns

`turnBy` keeps the routine readable without tracking absolute headings.

```cpp
void box() {
    clockwork::Motion motion(&chassis);
    chassis.setPose(0, 0, 0);

    for (int i = 0; i < 4; i++) {
        motion.driveDistance(24, 110, 2000);
        motion.turnBy(90);
    }
}
```

## 4. Intake with anti-jam

Poll the roller; kick backward briefly whenever it jams.

```cpp
void intake_with_antijam(int forMs) {
    std::uint32_t start = pros::millis();
    intake.in();
    while (pros::millis() - start < (std::uint32_t)forMs) {
        if (intake.stalled()) {
            intake.pulse(-127, 150); // clear the jam
            intake.in();
        }
        pros::delay(20);
    }
    intake.stop();
}
```

## 5. Timed ram to settle against a field element

Use `driveTimed` when you want a fixed push rather than a distance target.

```cpp
void settle_against_bar() {
    clockwork::Motion motion(&chassis);
    motion.driveTimed(500, 60); // 60 power forward for half a second
    intake.hold();              // clamp whatever we're holding
}
```

## 6. Intake with automatic anti-jam (the easy way)

`antiJam()` replaces the hand-rolled stall loop from example 4. Call it once per
loop right after commanding the intake. It clears jams on its own, without ever
blocking the rest of your code.

```cpp
// In opcontrol, or any control loop:
void run_intake_loop() {
    while (true) {
        intake.in();       // keep intaking
        intake.antiJam();  // auto-reverse a 200 ms burst if it jams, then resume
        pros::delay(20);
    }
}
```

You can tune the burst if your intake needs a harder or longer kick:

```cpp
intake.antiJam(127, 300, 200); // 127 power, 300 ms burst, declare jam after 200 ms stalled
```

## 7. Drive a lift/arm to a target with `PIDController`

lemlib controls the drivetrain, but not your other mechanisms. Use a
`PIDController` to hold an arm, lift, or flywheel at a target. Here an arm on a
rotation sensor moves to 120° and holds.

```cpp
#include "clockwork/clockwork.hpp"

pros::Motor arm(5);
pros::Rotation armSensor(6);

// kP=0.9 (main push), kI=0 (none needed), kD=4 (damp overshoot),
// integralCap=50, outputCap=127 (never exceed motor range).
clockwork::PIDController armPid(0.9f, 0.0f, 4.0f, 50.0f, 127.0f);

void move_arm_to(float targetDeg) {
    armPid.reset(); // clear history from any previous move
    while (!armPid.settled(2.0f)) {                 // within 2° and not moving
        float measured = armSensor.get_position() / 100.0f; // centidegrees -> deg
        float power = armPid.update(targetDeg - measured);  // error = target - measured
        arm.move(power);
        pros::delay(10); // steady loop timing matters, see the tuning guide
    }
    arm.brake();
}
```

To hold a position forever (e.g. keep an arm up under gravity), skip the
`settled()` exit and just keep calling `update()` every loop.

## 8. Autonomous selector

Register your routines, let the driver scroll them with the controller D-pad
before the match, and run the pick in `autonomous()`.

```cpp
#include "clockwork/clockwork.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
clockwork::AutonSelector selector(&master);

void left_rush();   // your routines, defined elsewhere
void right_safe();

void initialize() {
    selector.add("Left rush",  left_rush);
    selector.add("Right safe", right_safe);
    selector.add("Do nothing", [] {});
    selector.select("Right safe"); // optional default
    selector.draw();
}

void competition_initialize() {
    // Wait for the match to start while the driver picks. RIGHT/DOWN and
    // LEFT/UP on the D-pad move through the list.
    while (true) {
        selector.poll();
        pros::delay(20);
    }
}

void autonomous() {
    selector.run(); // runs whatever was selected
}
```

## 9. Slew-limited driver control

Ramp the drive so a hard joystick shove can't wheelie or brown out the robot.

```cpp
clockwork::SlewRateLimiter leftSlew(2.5f);  // ~0 to 127 over about half a second
clockwork::SlewRateLimiter rightSlew(2.5f);

void opcontrol() {
    while (true) {
        int l = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int r = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        left_mg.move(leftSlew.calculate(l));
        right_mg.move(rightSlew.calculate(r));
        pros::delay(10); // steady loop keeps the ramp consistent
    }
}
```

## 10. One-button pneumatic clamp

`Toggle` latches the button so one press clamps and the next press releases, and
`Pneumatics` drives the piston and remembers its state.

```cpp
clockwork::Pneumatics clamp('A'); // ADI port A
clockwork::Toggle clampToggle;

void opcontrol() {
    while (true) {
        bool held = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        clamp.set(clampToggle.update(held)); // press once to clamp, again to release
        pros::delay(10);
    }
}
```

## 11. Curved (expo) driver control

Shape the joystick so small movements are gentle and full pushes still hit full
power. Great for precise lineups without giving up top speed.

```cpp
void opcontrol() {
    while (true) {
        int y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // curve 2.0 for finesse near center, deadband 5 to kill stick drift
        left_mg.move(clockwork::joystickCurve(y + x, 2.0f, 5));
        right_mg.move(clockwork::joystickCurve(y - x, 2.0f, 5));
        pros::delay(10);
    }
}
```

## 12. Field-coordinate auton (point and heading moves)

`moveToPoint` and `turnToHeading` let you write a routine in field coordinates
instead of relative hops. Both block until they finish.

```cpp
void skills_open() {
    clockwork::Motion motion(&chassis);
    chassis.setPose(0, 0, 0);

    motion.moveToPoint(24, 24, 3000);   // drive to (24, 24)
    motion.turnToHeading(90);           // face 90 degrees
    motion.moveToPoint(48, 24, 2500);   // continue to (48, 24)
    motion.moveToPoint(24, 24, 2500, 100, false); // back up to (24, 24)
}
```

## 13. Flywheel held at speed

`FlywheelController` keeps a shooter at a target RPM and snaps back after each
shot drags it down.

```cpp
pros::Motor flywheel(11);
// kFF ~ 127/maxRpm gets it to speed; kP trims and recovers.
clockwork::FlywheelController fw(0.21f, 0.05f);

void run_flywheel(int targetRpm) {
    while (true) {
        flywheel.move(fw.update(targetRpm, flywheel.get_actual_velocity()));
        pros::delay(10);
    }
}
```

## 14. Smooth open-loop move with a trapezoidal profile

Follow a profile's position/velocity over time when you want a hand-tuned, smooth
move without full odometry.

```cpp
void nudge_forward() {
    clockwork::TrapezoidalProfile profile(24.0f, 40.0f, 80.0f); // 24 in, 40 in/s, 80 in/s^2
    std::uint32_t start = pros::millis();
    while (true) {
        float t = (pros::millis() - start) / 1000.0f; // seconds
        if (t >= profile.totalTime()) break;
        int power = (int)(profile.velocityAt(t) * (127.0f / 40.0f)); // vel -> power
        left_mg.move(power);
        right_mg.move(power);
        pros::delay(10);
    }
    left_mg.move(0);
    right_mg.move(0);
}
```
