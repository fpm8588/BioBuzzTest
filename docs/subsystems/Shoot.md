# Shoot

The shooter: one flywheel motor held at 2100 RPM. It runs on velocity (the motor controller holds the speed), not on power, so it stays near
2100 as the battery drains and when a game piece loads it.

- **Package:** `org.firstinspires.ftc.teamcode.subsystems`
- **Source:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shoot.java` (uses `VelocityMotor.java`, shared with the Intake)
- **Bench test:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TestShoot.java` (Driver Hub: "Test Shoot": a = spin, b or back = stop)
- **In the match:** Competition TeleOp, hold the operator's right trigger (`CompTeleOpControls.shoot`)

## Hardware

| Field | Type | Config name |
| --- | --- | --- |
| motor | DcMotorEx (FORWARD, RUN_USING_ENCODER, coasts when stopped) | `shoot` |

> The config name must match the Driver Hub configuration exactly, the encoder must be plugged in, and the motor type must be set
> (it gives the ticks per revolution; see docs/HARDWARE_CONFIG.md).

## Functions

- `spin()` - run at `SHOOT_RPM`
- `stop()` - cut power (the wheel coasts down)
- `getRpm()` - measured speed, RPM
- `isAtSpeed()` - true while spinning and within `AT_SPEED_TOLERANCE_RPM` of the target (the TeleOp screen shows READY)
- `problem()` - null, or what is wrong with the motor set-up (shown on the screen)

## Tuning (Panels)

- `SHOOT_RPM` = 2100
- `AT_SPEED_TOLERANCE_RPM` = 100
- `REVERSED` = false, `TICKS_PER_REV` = 0 (use the configured motor type) - both read when the Shoot is created, i.e. at INIT

## Notes

- The measured RPM on the screen is the check: it should settle at about 2100. If it sits well under 2100 with the trigger held, the motor
  cannot reach it (the screen warns when it can tell) or its ticks per revolution is wrong.
- Replaces the old two-wheel `Shooter` (`shoot1`, `shoot2`, ticks per second presets) and the shot-angle Adjustor, which are gone.
