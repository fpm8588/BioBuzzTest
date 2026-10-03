# Robot configuration chart (Driver Hub): drive motors + dead wheels only

This branch only uses the four drive motors, the two dead wheels and the Control Hub IMU. One Control Hub is
enough; no Expansion Hub is needed. Names are case-sensitive and are the original ones from the code.

## Control Hub

| Type | Port | Config name | Device | Direction set in code |
| --- | --- | --- | --- | --- |
| Motor | 0 | `lf` | Drive, front-left | REVERSE |
| Motor | 1 | `lb` | Drive, back-left | REVERSE |
| Motor | 2 | `rf` | Drive, front-right. **Forward dead wheel plugs into this port's encoder connector** | FORWARD |
| Motor | 3 | `rb` | Drive, back-right. **Strafe dead wheel plugs into this port's encoder connector** | FORWARD |
| I2C bus 0 | built-in | `imu` | Control Hub IMU (heading for Pedro) | n/a |

Nothing else needs to be configured. The code does not look up any other device.

Directions are set in code, so don't reverse anything in the config. Configure each drive motor as whichever
motor type you actually have.

## Step by step in the Driver Hub

1. Driver Hub: three-dot menu > **Configure Robot** > **New**.
2. Select the **Control Hub** (not the Expansion Hub).
3. **Motors** tab: set port 0 to `lf`, port 1 to `lb`, port 2 to `rf`, port 3 to `rb`, each with your motor type.
4. **I2C Bus 0**: make sure the built-in IMU is listed and named exactly `imu`. It is normally added for you.
5. Don't add servos, digital or analog devices, or any Ethernet / USB device.
6. **Save** with a name (for example `DriveOnly`), then **Activate** it.

## Dead wheels (goBILDA 4-bar pods)

| Pod | Plugs into | Pedro name in `Constants.java` | Counts up when the robot moves |
| --- | --- | --- | --- |
| Forward pod | Control Hub encoder port **2** (`rf`) | `forwardEncoder_HardwareMapName("rf")` | forward |
| Strafe pod | Control Hub encoder port **3** (`rb`) | `strafeEncoder_HardwareMapName("rb")` | left |

1. Unplug the drive motors' own encoder cables from encoder ports 2 and 3. Pedro's mecanum drive doesn't use them.
   Leave ports 0 and 1 alone.
2. Plug each pod's cable into the encoder connector of its port. The motor power wires stay on the same port.
3. `rf` and `rb` are the two drive motors with direction FORWARD, so the SDK doesn't flip their encoder sign.
4. Not checked from here: the pod cable's pinout and supply voltage against a REV encoder port. Confirm in the
   goBILDA pod manual before powering up.
5. `RobotHealthMonitor` reads velocity on every motor, so on `rf` and `rb` it reports the pod's velocity.
   Ignore stall warnings from those two.

## What is switched off in this branch

`CompTeleOp` and `CompAuto` have the shooter, intake, lift, sorter and Limelight code commented out (marked
`DISABLED`). The subsystem classes are untouched and just aren't created. The `Test ...` subsystem OpModes carry
`@Disabled` so they are hidden from the Driver Hub menu. To bring a subsystem back, uncomment its lines and add its
devices to the config from the table below.

| Config name | Type | Subsystem |
| --- | --- | --- |
| `inOne`, `inTwo` | motors | Intake |
| `spinOne`, `spinTwo` | motors (keep encoders plugged in) | Shooter |
| `lift` | servo | Lift |
| `sortOne`, `sortTwo` | servos | Sorter |
| `limelight` | Limelight 3A (Ethernet Devices) | Limelight |

Eight motors need an Expansion Hub as well.

## Check it works (OpMode "Odometry Test (push by hand)", group Odometry)

This repo has no Pedro tuner OpModes, so use this one. It floats the drive wheels so you can push the robot by
hand, and shows Pedro's pose beside the raw pod counts. Press **A** to zero before each test.

1. Push the robot forward: x must rise. Push it left: y must rise. If one is backwards, flip that pod in
   `Constants.java` (`forwardEncoderDirection` / `strafeEncoderDirection`, `Encoder.FORWARD` or `Encoder.REVERSE`).
2. Turn the robot left (counter-clockwise from above): heading must increase. If not, fix the IMU orientation in
   `Constants.java` to match how the Control Hub is mounted.
3. Spin the robot in place: x and y should hardly move. If they trace a circle, an offset is wrong. See
   `ODOMETRY.md`.
4. Ticks to inches: mark out a known distance (for example 48 in), push the robot along it, and read the suggested
   `ticksToInches` on screen. Set it in `Constants.java` for that pod. Use D-pad up/down if your distance isn't 48.

Competition TeleOp and Competition Auto also run in this build (drive only).
