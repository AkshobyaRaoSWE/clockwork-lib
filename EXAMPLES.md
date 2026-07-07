# Examples

Full routines using the CLOCKWORK library. In each, `chassis` is a configured
`lemlib::Chassis` and `intake` is a `clockwork::Roller` wrapping your intake
motor group — both declared as globals in your own `robot.cpp`.

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
        // squared against the wall — reset the known axis
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
loop right after commanding the intake — it clears jams on its own, without ever
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
        pros::delay(10); // steady loop timing matters — see the tuning guide
    }
    arm.brake();
}
```

To hold a position forever (e.g. keep an arm up under gravity), skip the
`settled()` exit and just keep calling `update()` every loop.
