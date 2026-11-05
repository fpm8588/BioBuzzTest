# Robot configuration chart (Driver Hub)

The drive (four motors, two dead wheels, IMU) is on the Control Hub. The auxiliary systems (intake, the two shooter
wheels, the shot-angle adjustor) need the devices in the second table below, and the Control Hub's four motor ports
are all taken by the drive, so **the intake and shooter motors go on an Expansion Hub**. Names are case-sensitive and are
the ones in the code.

## Control Hub

| Type | Port | Config name | Device | Direction set in code |
| --- | --- | --- | --- | --- |
| Motor | 0 | `lf` | Drive, front-left | REVERSE |
| Motor | 1 | `lb` | Drive, back-left | REVERSE |
| Motor | 2 | `rf` | Drive, front-right. **Forward dead wheel plugs into this port's encoder connector** | FORWARD |
| Motor | 3 | `rb` | Drive, back-right. **Strafe dead wheel plugs into this port's encoder connector** | FORWARD |
| I2C bus 0 | built-in | `imu` | Control Hub IMU (heading for Pedro) | n/a |

Nothing else needs to be configured for driving. The auxiliary systems (intake, shooter, adjustor; see **Auxiliary systems** below)
are **switched off** (`AuxSettings.ENABLED = false`), so those devices need not be in the configuration. When they are switched
back on, Competition TeleOp, Competition Auto and Shoot On Tags **fail at INIT if the devices are not in the active configuration**. The Limelight is only used by the vision OpModes (next section).

## Limelight 3A (vision OpModes, and goal-tag aiming in Competition TeleOp / Auto)

| Type | Plugs into | Config name | Notes |
| --- | --- | --- | --- |
| Limelight 3A (appears as a USB / Ethernet device) | Control Hub blue **USB 3.0** port | `limelight` | Scan in Configure Robot, tap its name and rename it exactly `limelight`. |

Full camera setup is in `LIMELIGHT_POLLEN.md` (pollen) and `LIMELIGHT_APRILTAG.md` (goal tag). Competition TeleOp and Auto use it for aiming but still run without it (aiming is off); the other OpModes do not need it.

Directions are set in code, so don't reverse anything in the config. Configure each drive motor as whichever
motor type you actually have.

## Step by step in the Driver Hub

1. Driver Hub: three-dot menu > **Configure Robot** > **New**.
2. Select the **Control Hub** (not the Expansion Hub).
3. **Motors** tab: set port 0 to `lf`, port 1 to `lb`, port 2 to `rf`, port 3 to `rb`, each with your motor type.
4. **I2C Bus 0**: make sure the built-in IMU is listed and named exactly `imu`. It is normally added for you.
5. For the auxiliary systems add the **Expansion Hub** and the devices from the **Auxiliary systems** table below (`intake`, `shoot1`, `shoot2`, `adjustor`). Don't add anything else, unless you are running the vision OpModes.
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

## Auxiliary systems (intake, shooter, shot-angle adjustor): CURRENTLY DISABLED

**They are switched off:** `subsystems/AuxSettings.java` has `ENABLED = false`. While it is false the `Intake`, `Shooter` and `Adjustor` classes do not look up
any hardware and every command does nothing, so the four devices below do not have to be in the configuration, and the OpModes that use them (the TeleOps,
Competition Auto, Shoot On Tags, the three bench tests) still run (Shoot On Tags drives to the shooting position and does not shoot; the bench tests say DISABLED).
**To bring them back:** set `ENABLED = true`, rebuild, and add the Expansion Hub and these devices to the configuration:

| Config name | Type | Subsystem | Notes |
| --- | --- | --- | --- |
| `intake` | motor | `Intake` | Run on power. `Intake.INTAKE_REVERSED` flips it if "in" spins the wrong way. |
| `shoot1` | motor, **encoder plugged in** | `Shooter` | Run on velocity (the controller holds the speed). Set REVERSE in code. |
| `shoot2` | motor, **encoder plugged in** | `Shooter` | Faces `shoot1`, so it is set FORWARD while `shoot1` is REVERSE: the wheels spin opposite ways and both are commanded a positive velocity. |
| `adjustor` | servo | `Adjustor` | Shot angle. Positions 0 to 1, limited by `Adjustor.MIN_POSITION` / `MAX_POSITION`. |

Port numbers are your choice; suggested: `intake`, `shoot1`, `shoot2` on Expansion Hub motor ports 0, 1, 2, and `adjustor` on a
servo port of either hub. Directions are set in code: don't reverse anything in the config. If a shooter wheel throws the wrong way, flip
`Shooter.SHOOT_ONE_REVERSED` / `SHOOT_TWO_REVERSED`. The shooter speeds are in **ticks per second** (`Shooter.LOW_VELOCITY` 1100,
`HIGH_VELOCITY` 1400, the values from last year), set with `setVelocity`, so what they mean in RPM depends on the motor and any gearing.

Still switched off in `CompTeleOpBase` (the red and blue TeleOps) and `CompAuto` (their code is commented out and marked `DISABLED`): the **sorter**
(`sortOne`, `sortTwo` servos) and the **Limelight** (`limelight`, Ethernet device). To bring one back, uncomment its lines and add its
devices to the config.

Seven motors (four drive, three auxiliary) need an Expansion Hub as well.

## Check it works and tune

Pedro's tuning menu is in the project: `TeamCode/.../pedroPathing/Tuning.java`, copied unchanged from the official
Pedro Quickstart at its last commit for `com.pedropathing:ftc:2.1.2` (commit `4d160da`). On the Driver Hub pick the
**Tuning** OpMode (group "Pedro Pathing"). It builds the robot through `Constants.createFollower(hardwareMap)`, so it
uses the drive motors, dead wheels and IMU configured here and nothing else.

Menu: Localization (Localization Test, Offsets Tuner, Forward / Lateral / Turn tuners), Automatic (velocity,
zero-power acceleration, Predictive Braking), Manual (Translational, Heading, Drive, Centripetal), Tests (Line,
Triangle, Circle). The Turn Tuner isn't needed with this two-wheel + IMU setup.

**Full step-by-step guide: `TUNING.md`.** Competition TeleOp and Competition Auto also run in this build (drive only).
