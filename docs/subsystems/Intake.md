<!-- @ftc-toolchain generated: subsystem-doc — scaffolded; team edits expected -->
# Intake

Single-motor intake with forward/reverse control. Works the same from any OpMode, autonomous or teleop.

- **Package:** `org.firstinspires.ftc.teamcode.subsystems`
- **Source:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Intake.java`
- **Bench test:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TestIntake.java` (Driver Station: "Test Intake")

## Hardware

| Field | Type | Config name |
| --- | --- | --- |
| `intake` | DcMotorEx | `intake` |

> Config names must match the robot configuration on the Driver Station exactly.

## Functions

- `intakeIn()` — run inward at `INTAKE_POWER`
- `intakeOut()` — run outward (reject) at `INTAKE_POWER`
- `stop()` — cut power

## Tuning

- `INTAKE_POWER` = 0.6 — default intake power (dashboard-tunable)
- `INTAKE_REVERSED` = false — flip if "in" spins the wrong way (read when created)

## Notes / quirks

- The old `inOne` / `inTwo` motors are gone.
