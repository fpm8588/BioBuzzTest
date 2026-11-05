<!-- @ftc-toolchain generated: subsystem-doc — scaffolded; team edits expected -->
# Shooter

Flywheel shooter with two velocity presets (test build; roles to be retuned when DECODE game pieces are known)

- **Package:** `org.firstinspires.ftc.teamcode.subsystems`
- **Source:** `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\Shooter.java`
- **Bench test:** `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\TestShooter.java` (Driver Station: "Test Shooter")

## Hardware

| Field | Type | Config name |
| --- | --- | --- |
| `spinOne` | DcMotorEx (reversed) | `spinOne` |
| `spinTwo` | DcMotorEx (reversed) | `spinTwo` |

> Config names must match the robot configuration on the Driver Station exactly.

## Functions

- `spinAtLow()` — TODO: describe
- `spinAtHigh()` — TODO: describe
- `stop()` — cut power to all actuators

## Tuning

- `LOW_VELOCITY` = 1100 — ticks/sec, close-range shot preset (dashboard-tunable)
- `HIGH_VELOCITY` = 1400 — ticks/sec, far-range shot preset (dashboard-tunable)

## Notes / quirks

_TODO: wiring notes, gotchas, mechanical constraints._
