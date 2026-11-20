# Robot configuration chart (Driver Hub)

The drive (four motors, two dead wheels, IMU) is on the Control Hub. The two mechanisms (the intake and the shoot flywheel) need the
motors in the **Intake and Shoot** table below, and the Control Hub's four motor ports are all taken by the drive, so **the intake and
shoot motors go on an Expansion Hub**. Names are case-sensitive and are the ones in the code.

## Control Hub

| Type | Port | Config name | Device | Direction set in code |
| --- | --- | --- | --- | --- |
| Motor | 0 | `lf` | Drive, front-left | FORWARD |
| Motor | 1 | `lb` | Drive, back-left | FORWARD |
| Motor | 2 | `rf` | Drive, front-right. **Forward dead wheel plugs into this port's encoder connector** | REVERSE |
| Motor | 3 | `rb` | Drive, back-right. **Strafe dead wheel plugs into this port's encoder connector** | REVERSE |
| I2C bus 0 | built-in | `imu` | Control Hub IMU (heading for Pedro) | n/a |

Nothing else needs to be configured for driving. Competition TeleOp **fails at INIT if `intake` and `shoot` (see **Intake and Shoot** below) are not in the active configuration**. The Limelight is only used by the vision OpModes (next section).

## Limelight 3A (only for Pollen Vision Test and Pollen Chase)

| Type | Plugs into | Config name | Notes |
| --- | --- | --- | --- |
| Limelight 3A (appears as a USB / Ethernet device) | Control Hub blue **USB 3.0** port | `limelight` | Scan in Configure Robot, tap its name and rename it exactly `limelight`. |

Full camera setup is in `LIMELIGHT_POLLEN.md`. The other OpModes do not need it.

Directions are set in code (all four are the opposite of Pedro's defaults on this robot: `lf` and `lb` FORWARD, `rf` and `rb` REVERSE), so don't reverse anything in the config. Configure each drive motor as whichever
motor type you actually have.

## Step by step in the Driver Hub

1. Driver Hub: three-dot menu > **Configure Robot** > **New**.
2. Select the **Control Hub** (not the Expansion Hub).
3. **Motors** tab: set port 0 to `lf`, port 1 to `lb`, port 2 to `rf`, port 3 to `rb`, each with your motor type.
4. **I2C Bus 0**: make sure the built-in IMU is listed and named exactly `imu`. It is normally added for you.
5. For the mechanisms add the **Expansion Hub** and the two motors from the **Intake and Shoot** table below (`intake`, `shoot`). Don't add anything else, unless you are running the vision OpModes.
6. **Save** with a name (for example `DriveOnly`), then **Activate** it.

## Dead wheels (goBILDA 4-bar pods)

| Pod | Plugs into | Pedro name in `Constants.java` | Counts up when the robot moves |
| --- | --- | --- | --- |
| Forward pod | Control Hub encoder port **2** (`rf`) | `forwardEncoder_HardwareMapName("rf")` | forward |
| Strafe pod | Control Hub encoder port **3** (`rb`) | `strafeEncoder_HardwareMapName("rb")` | left |

1. Unplug the drive motors' own encoder cables from encoder ports 2 and 3. Pedro's mecanum drive doesn't use them.
   Leave ports 0 and 1 alone.
2. Plug each pod's cable into the encoder connector of its port. The motor power wires stay on the same port.
3. Why a drive-motor port: the pods have no motor of their own, so each shares a drive motor's encoder connector.
   Pedro's encoder wrapper accounts for the motor's configured direction itself, so which motor you share with
   does not change the pod's direction logic. Flip a pod with `Encoder.REVERSE` if the Localization Test says so.
   Pedro's two-wheel docs say REV encoder ports **0 and 3** are the fastest and recommend them for both pods.
   Optional: move the forward pod to port 0 (`lf`) and set `forwardEncoder_HardwareMapName("lf")`.
4. Not checked from here: the pod cable's pinout and supply voltage against a REV encoder port. Confirm in the
   goBILDA pod manual before powering up.
5. `RobotHealthMonitor` reads velocity on every motor, so on `rf` and `rb` it reports the pod's velocity.
   Ignore stall warnings from those two.

## Intake and Shoot

| Config name | Type | Subsystem | Notes |
| --- | --- | --- | --- |
| `intake` | motor, **encoder plugged in** | `Intake` | Held at 900 RPM (velocity control). `Intake.REVERSED` flips it if "in" spins the wrong way. |
| `shoot` | motor, **encoder plugged in** | `Shoot` | Held at 2100 RPM (velocity control). `Shoot.REVERSED` flips it if it throws the wrong way. |

Both are on an Expansion Hub motor port (suggested: `intake` port 0, `shoot` port 1). Directions are set in code: don't reverse anything in the config.

**Choose the right motor type for each in the configuration** (for example "goBILDA 5203 series" or "REV Core Hex"). The code turns RPM into
the encoder ticks per second the motor controller needs using that type's ticks per revolution, so a wrong type gives the wrong RPM. If the type
is "unspecified" the code refuses to run that motor and the screen says so; you can instead type the number into `Shoot.TICKS_PER_REV` /
`Intake.TICKS_PER_REV` (encoder ticks per revolution of the motor shaft; 0 = use the configured type). The screen also warns if the speed is
more than the motor can reach (the SDK treats about 85 % of a motor's rated free speed as the most it can hold under load).

The **Limelight** (`limelight`, Ethernet device) is only used by the vision OpModes; add it to the config only if you run those.

Six motors (four drive, two mechanisms) need an Expansion Hub as well.

## Check it works and tune

Pedro's tuning menu is in the project: `TeamCode/.../pedroPathing/Tuning.java`, copied unchanged from the official
Pedro Quickstart at its last commit for `com.pedropathing:ftc:2.1.2` (commit `4d160da`). On the Driver Hub pick the
**Tuning** OpMode (group "Pedro Pathing"). It builds the robot through `Constants.createFollower(hardwareMap)`, so it
uses the drive motors, dead wheels and IMU configured here and nothing else.

Menu: Localization (Localization Test, Offsets Tuner, Forward / Lateral / Turn tuners), Automatic (velocity,
zero-power acceleration, Predictive Braking), Manual (Translational, Heading, Drive, Centripetal), Tests (Line,
Triangle, Circle). The Turn Tuner isn't needed with this two-wheel + IMU setup.

**Full step-by-step guide: `TUNING.md`.** Competition TeleOp and Competition Auto also run in this build (drive only).
