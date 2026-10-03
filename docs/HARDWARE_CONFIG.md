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

## Check it works (Pedro's own tuners)

This build uses Pedro's standard `Constants.createFollower(hardwareMap)`, so Pedro's tuning OpModes work with it
unchanged: Localization Test, Forward / Lateral / Turn tuners (these also give the dead-wheel ticks-to-inches),
the velocity and zero-power-acceleration tuners, and the PID tests.

**They are not in this repo yet.** Pedro ships them in its Quickstart project as `Tuning.java` (in the
`pedroPathing` package). Copy that file from a Quickstart whose version matches `com.pedropathing:ftc:2.1.2`
(see `build.dependencies.gradle`) into `TeamCode/.../pedroPathing/`. If the Quickstart you grab is for a newer Pedro
(3.x), its tuner code won't match 2.1.2: use the Quickstart release for 2.x, or upgrade the dependency and adjust
`Constants.java` together.

Then run them in this order (full guide: https://pedropathing.com/docs/pathing/tuning):

1. **Localization Test:** push the robot forward and x must rise. Push it left and y must rise. Turn it left
   (counter-clockwise from above) and heading must increase. If a pod counts backwards, flip its `Encoder`
   direction in `Constants.java`. If heading is backwards, fix the IMU orientation there.
2. Spin the robot in place: x and y should hardly move. If they trace a circle, an offset is wrong.
   See `ODOMETRY.md`.
3. **Forward Tuner** and **Lateral Tuner:** set `forwardTicksToInches` / `strafeTicksToInches`.
4. **Turn Tuner**, then the velocity, zero-power-acceleration and PID tuners.

Competition TeleOp and Competition Auto also run in this build (drive only).
