# RobotHealthMonitor

Live power/health dashboard: battery voltage + estimated charge, per-motor current draw, and
simple anomaly detection (stalls, brownouts, noisy/intermittent connections). Surfaces to the
Driver Station, the [Panels dashboard](https://panels.bylazar.com) at `192.168.43.1:8001`
(connect to robot wifi), and a CSV event log for post-match review. Paired with `Drawing`
for live field position.

- **Package:** `org.firstinspires.ftc.teamcode.util`
- **Source:** `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\util\RobotHealthMonitor.java`
- **Field view:** `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\util\Drawing.java`
- **Wired into:** `CompTeleOp.java`, `CompAuto.java`

## Hardware

No dedicated hardware config names — it auto-discovers every `DcMotorEx` and `VoltageSensor`
already registered in the hardware map (works unchanged as motors are added/renamed).

## Usage

```java
health = new RobotHealthMonitor(hardwareMap, telemetry, "CompTeleOp");
Drawing.init(); // once, at init

// each loop:
health.update(driver);        // pass a Gamepad to get a rumble on new warnings, or update() for none
Drawing.drawRobot(follower.getPose());       // TeleOp: just the live pose
Drawing.drawDebug(follower);                 // Auto: pose + current path
```

## What it reports

- **Battery:** min voltage across all voltage sensors, plus an estimated charge % from a
  tunable voltage curve (`BATTERY_CURVE_VOLTAGE` / `BATTERY_CURVE_PERCENT`). This is a rough
  trend indicator, not a precise fuel gauge — voltage sags under load.
- **Per-motor current + velocity:** graphed live on the Panels dashboard under `motors/<name>/...`.
- **Warnings** (Driver Station + Panels + logged to `stats/health_events.csv` on first occurrence):
  - `BROWNOUT` — battery voltage at/below `BROWNOUT_VOLTAGE`.
  - `BATTERY CRITICAL` / `Battery low` — estimated charge below `CRITICAL_BATTERY_PERCENT` / `LOW_BATTERY_PERCENT`.
  - `STALL: <motor>` — commanded power but high current + ~0 velocity sustained for `STALL_TIME_SEC`.
    Likely a mechanical bind, or a disconnected/broken encoder if it never clears.
  - `NOISY CURRENT: <motor>` — current swinging by more than `NOISY_CURRENT_STDDEV_AMPS` across
    the rolling sample window. Likely a loose connector or a failing motor.

## Tuning

All thresholds are `@Configurable` (dashboard-tunable) starting guesses — retune once you've
watched real match data for your motors and packs:

- `STALL_CURRENT_AMPS` = 5.0, `STALL_VELOCITY_TICKS_PER_SEC` = 5.0, `STALL_TIME_SEC` = 0.75
- `NOISY_CURRENT_STDDEV_AMPS` = 2.5
- `LOW_BATTERY_PERCENT` = 30, `CRITICAL_BATTERY_PERCENT` = 15, `BROWNOUT_VOLTAGE` = 9.5

## Notes / quirks

- The battery curve assumes a REV 12V pack at rest; under load the same physical charge reads
  as a lower voltage, so expect the % to dip during high-draw moments (shooter spin-up, stalls)
  even without real charge loss.
- Stall detection can't tell a jammed mechanism apart from a disconnected encoder — both look
  like "high current, no velocity." Check the physical mechanism first.
- `Drawing`'s API calls were verified against the real `fullpanels`/`pedropathing` artifacts
  (decompiled with `javap` since this dev environment has no Android SDK to run a full build),
  but a real Android Studio build is still worth doing before you trust it on the robot.
