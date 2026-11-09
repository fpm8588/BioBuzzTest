<!-- @ftc-toolchain generated: subsystem-doc — scaffolded; team edits expected -->
# Adjustor

Single-servo shot-angle adjustor with three preset positions and free positioning. Replaces the old Lift. Works the same from any
OpMode, autonomous or teleop.

- **Package:** `org.firstinspires.ftc.teamcode.subsystems`
- **Source:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Adjustor.java`
- **Bench test:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TestAdjustor.java` (Driver Station: "Test Adjustor")

## Hardware

| Field | Type | Config name |
| --- | --- | --- |
| `adjustor` | Servo | `adjustor` |

> Config names must match the robot configuration on the Driver Station exactly.

## Functions

- `goHigh()` / `goMid()` / `goLow()` — move to a preset
- `setPosition(p)` — move to any position, limited to `MIN_POSITION`..`MAX_POSITION`
- `nudge(delta)` — move by delta from where it was last told to go
- `getPosition()` — where it was last told to go (NaN before the first command; a servo gives no feedback)

## Tuning

- `POSITION_HIGH` = 1.0, `POSITION_MID` = 0.5, `POSITION_LOW` = 0.0 (dashboard-tunable)
- `MIN_POSITION` = 0.0, `MAX_POSITION` = 1.0 — the servo never goes outside these

## Notes / quirks

- Which position gives which shot angle is not known yet: set the presets on the robot.
- Teleop (operator): X = high, A = mid, B = low.
