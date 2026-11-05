<!-- @ftc-toolchain generated: subsystem-doc — scaffolded; team edits expected -->
# Intake

Dual-motor intake with forward/reverse control

- **Package:** `org.firstinspires.ftc.teamcode.subsystems`
- **Source:** `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\Intake.java`
- **Bench test:** `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\TestIntake.java` (Driver Station: "Test Intake")

## Hardware

| Field | Type | Config name |
| --- | --- | --- |
| `inOne` | DcMotorEx | `inOne` |
| `inTwo` | DcMotorEx (reversed) | `inTwo` |

> Config names must match the robot configuration on the Driver Station exactly.

## Functions

- `intakeIn()` — TODO: describe
- `intakeOut()` — TODO: describe
- `stop()` — cut power to all actuators

## Tuning

- `INTAKE_POWER` = 0.6 — default intake power (dashboard-tunable)

## Notes / quirks

_TODO: wiring notes, gotchas, mechanical constraints._
