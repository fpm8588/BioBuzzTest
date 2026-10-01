# Robot configuration chart (Driver Hub)

Names must match the code exactly (case-sensitive). Assumes one Control Hub + one Expansion Hub, because the
code drives 8 motors. Every name below comes from the code (`Constants.java` and `subsystems/*.java`).

## Control Hub

| Type | Port | Config name | Device | Direction set in code |
| --- | --- | --- | --- | --- |
| Motor | 0 | `lf` | Drive, front-left | REVERSE |
| Motor | 1 | `lb` | Drive, back-left | REVERSE |
| Motor | 2 | `rf` | Drive, front-right. **Forward dead wheel encoder plugs into this port's encoder connector** | FORWARD |
| Motor | 3 | `rb` | Drive, back-right. **Strafe dead wheel encoder plugs into this port's encoder connector** | FORWARD |
| I2C bus 0 | built-in | `imu` | Control Hub IMU (heading for Pedro) | n/a |
| Ethernet / USB device | Control Hub USB 3.0 port | `limelight` | Limelight 3A (type Limelight3A; appears under Ethernet Devices after Scan) | n/a |

## Expansion Hub

| Type | Port | Config name | Device | Direction set in code |
| --- | --- | --- | --- | --- |
| Motor | 0 | `inOne` | Intake motor 1 | FORWARD |
| Motor | 1 | `inTwo` | Intake motor 2 | REVERSE |
| Motor | 2 | `spinOne` | Shooter flywheel 1 (**keep its own encoder plugged in**) | REVERSE |
| Motor | 3 | `spinTwo` | Shooter flywheel 2 (**keep its own encoder plugged in**) | REVERSE |
| Servo | 0 | `lift` | Lift servo | n/a |
| Servo | 1 | `sortOne` | Sorter gate 1 | n/a |
| Servo | 2 | `sortTwo` | Sorter gate 2 | n/a |

Directions are set in code, so configure the motors as plain motors with the right type for each (drive,
intake, shooter) and don't try to reverse anything in the config.

## Dead wheels (goBILDA 4-bar pods, no Pinpoint)

| Pod | Plugs into | Pedro name in `Constants.java` | Counts up when the robot moves |
| --- | --- | --- | --- |
| Forward pod | Control Hub encoder port **2** | `forwardEncoder_HardwareMapName("rf")` | forward |
| Strafe pod | Control Hub encoder port **3** | `strafeEncoder_HardwareMapName("rb")` | left |

1. Unplug the drive motors' own encoder cables from encoder ports 2 and 3. Pedro's mecanum drive doesn't use them.
   Leave ports 0 and 1 alone.
2. Plug each pod's cable into the encoder connector of its port. The motor power wires stay on the same port.
3. Why these ports: `rf` and `rb` are the two drive motors with direction FORWARD, so the SDK doesn't flip their
   encoder sign. The shooter ports are excluded because `Shooter` reads their velocity.
4. Not checked from here: the pod cable's pinout and supply voltage against a REV encoder port. Confirm in the
   goBILDA pod manual before powering up.
5. `RobotHealthMonitor` reads velocity on every motor, so on `rf` and `rb` it will report the pod's velocity.
   Ignore stall warnings from those two.

## Check it works

1. Open the Pedro **Localization Test**. Push the robot forward by hand: x must rise. Push it left: y must rise.
   If one is backwards, flip that pod in `Constants.java` (`forwardEncoderDirection` /
   `strafeEncoderDirection`, `Encoder.FORWARD` or `Encoder.REVERSE`).
2. Turn the robot left (counter-clockwise from above): heading must increase. If not, fix the IMU orientation
   in `Constants.java` to match how the Control Hub is mounted.
3. Spin the robot in place: x and y should hardly move. If they trace a circle, an offset is wrong. See
   `ODOMETRY.md`.
