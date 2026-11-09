<!-- @ftc-toolchain generated: subsystem-doc — scaffolded; team edits expected -->
# Shooter

Two-wheel flywheel shooter. `shoot1` and `shoot2` face each other, so they spin in opposite directions; both are run on velocity
(the motor controller holds the speed), not on power. Works the same from any OpMode, autonomous or teleop.

- **Package:** `org.firstinspires.ftc.teamcode.subsystems`
- **Source:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java`
- **Bench test:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TestShooter.java` (Driver Station: "Test Shooter")

## Hardware

| Field | Type | Config name |
| --- | --- | --- |
| `shootOne` | DcMotorEx (REVERSE, RUN_USING_ENCODER) | `shoot1` |
| `shootTwo` | DcMotorEx (FORWARD, RUN_USING_ENCODER) | `shoot2` |

> Config names must match the robot configuration on the Driver Station exactly. Keep both encoders plugged in.

## Functions

- `spinAtLow()` — both wheels to `LOW_VELOCITY` (close-range shot)
- `spinAtHigh()` — both wheels to `HIGH_VELOCITY` (far-range shot)
- `setVelocity(ticksPerSec)` — both wheels to any velocity (positive = throws the game piece out)
- `getVelocity()` — average of the two wheels, ticks/sec
- `stop()` — cut power to both wheels

## Tuning

- `LOW_VELOCITY` = 1100 — ticks/sec, close-range shot preset (dashboard-tunable)
- `HIGH_VELOCITY` = 1400 — ticks/sec, far-range shot preset (dashboard-tunable)
- `SHOOT_ONE_REVERSED` = true, `SHOOT_TWO_REVERSED` = false — flip a wheel that throws the wrong way (read when created)

## Notes / quirks

- Velocity is in encoder ticks per second, not RPM. RPM = ticks/sec / (ticks per revolution of the wheel) x 60.
- The old `spinOne` / `spinTwo` motors (both the same direction) are gone.
