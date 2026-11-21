# Intake

The intake: one motor held at 900 RPM, in or out. It runs on velocity (the motor controller holds the speed), not on power.

- **Package:** `org.firstinspires.ftc.teamcode.subsystems`
- **Source:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Intake.java` (uses `VelocityMotor.java`, shared with the Shoot)
- **Bench test:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TestIntake.java` (Driver Hub: "Test Intake": a = in, b = out, back = stop)
- **In the match:** Competition TeleOp, hold the driver's (controller 1) left trigger (in) or D-pad left (out / reject) (`CompTeleOpControls`)

## Hardware

| Field | Type | Config name |
| --- | --- | --- |
| motor | DcMotorEx (FORWARD, RUN_USING_ENCODER, brakes when stopped) | `intake` |

> The config name must match the Driver Hub configuration exactly, the encoder must be plugged in, and the motor type must be set
> (it gives the ticks per revolution; see docs/HARDWARE_CONFIG.md).

## Functions

- `intakeIn()` - run inward at `INTAKE_RPM`
- `intakeOut()` - run outward at `INTAKE_RPM`
- `stop()` - cut power
- `getRpm()` - measured speed, RPM (negative = outward)
- `problem()` - null, or what is wrong with the motor set-up (shown on the screen)

## Tuning (Panels)

- `INTAKE_RPM` = 900
- `REVERSED` = false, `TICKS_PER_REV` = 0 (use the configured motor type) - both read when the Intake is created, i.e. at INIT

## Notes

- 900 RPM at the motor shaft needs a motor that can reach it. A motor with a gearbox rated around 435 RPM or less cannot (the screen warns).
  Check the measured RPM on the screen.
- Replaces the old power-controlled intake (0.6 power).
