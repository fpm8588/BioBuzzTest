<!-- @ftc-toolchain generated: subsystem-doc — scaffolded; team edits expected -->
# Limelight

Limelight 3A vision subsystem for AprilTag localization / target tracking (pipeline TBD once season goal known)

- **Package:** `org.firstinspires.ftc.teamcode.subsystems`
- **Source:** `TeamCode\src\main\java\org\firstinspires\ftc\teamcode\subsystems\Limelight.java`

## Hardware

| Field | Type | Config name |
| --- | --- | --- |
| `limelight` | Limelight3A (USB / Ethernet device) | `limelight` |

> Config names must match the robot configuration on the Driver Station exactly.

## Functions

- `selectPipeline(index, timeoutMs)` — switch pipeline and wait until the Limelight reports it is running
- `getPollen(minArea, maxStalenessMs, haveLast, lastTx, stickyDeg)` — the yellow pollen to go for (angles, area, frame age), or `null`
- `getBlobs(maxStalenessMs)` — every colour blob (tx, ty, area) in the latest frame; used for obstacle (opposing robot) detection
- `statusLine()` / `getStalenessMs()` — camera health for telemetry
- `getTargetX()` — TODO: describe
- `getTargetY()` — TODO: describe
- `hasTarget()` — TODO: describe
- `getBotPose()` — TODO: describe
- `stop()` — cut power to all actuators

## Tuning

_TODO: record PID values, RPM setpoints, servo positions, etc. as you tune._

## Notes / quirks

- Plug into the Control Hub blue USB 3.0 port. Setup, pipeline and calibration: `docs/LIMELIGHT_POLLEN.md`.
- `getLatestResult()` is never null since SDK 10.3; check `isValid()` and the staleness instead.
- After a pipeline switch, results from the old pipeline keep arriving for a moment: `selectPipeline()` waits for the switch.
