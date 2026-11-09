# Pedro Pathing tuning guide (drive + two dead wheels + Control Hub IMU)

Written against the project's Pedro version, `com.pedropathing:ftc:2.1.2`, and checked against the Pedro 2.1.2
library source, the official docs from the same period, and the `Tuning.java` copy in this project
(`TeamCode/.../pedroPathing/Tuning.java`). Every value you tune is pasted into
`TeamCode/.../pedroPathing/Constants.java`.

Official guide for reference: https://pedropathing.com/docs/pathing/tuning

## Read first

- **Everything in `Constants.java` marked TODO is a placeholder, not a tuned value** (mass, velocities,
  zero-power accelerations, PIDFs, pod offsets, ticks-to-inches). The tuners replace them.
- **Tune on the finished robot.** Localization checks (directions, ticks-to-inches) are valid on the bare chassis.
  Everything about speed, braking and PIDFs depends on the robot's mass and weight distribution. When the intake,
  shooter, battery etc. are added, re-run the velocity tuners, the braking/PIDF tuners and the Offsets Tuner.
- **Panels edits are temporary.** Values changed on the Panels dashboard, or applied with the A button in a
  tuner, are lost when the robot restarts. Always copy the final number into `Constants.java`.
- **Do localization first and finish it.** Pedro's own docs: if localization is wrong, every later tuner gives
  wrong answers or the robot drives off forever.

## 0. Before you start

**Setup**
- Build and install the project. Fully charge the battery.
- Use a flat surface, ideally field tiles (same surface you will compete on). Clear at least 4 to 5 feet in every
  direction for the moving tuners. Have a tape measure. A field tile is 24 inches.
- Plug a gamepad into the Driver Hub and press **Start + A** so it registers as gamepad 1.
- For live tuning, connect a laptop to the robot's wifi and open **Panels** at `192.168.43.1:8001`.
  If the Configurables list is empty, select an OpMode in Tuning with the gamepad first, then reload the page.

**How the Tuning menu works** (OpMode **Tuning**, group "Pedro Pathing")
1. Select **Tuning** on the Driver Hub and press **INIT**. The menu appears on the screen.
2. **D-pad up/down** moves the cursor. **Right bumper or D-pad right** selects. **Left bumper or D-pad left**
   goes back.
3. After picking a tuner, press **START** to run it. (START with nothing selected throws "No OpMode selected!".)
4. To run a different tuner, press **STOP**, then INIT again.
5. **Gamepad 1 B** stops the automatic tuners (velocity, zero-power, predictive braking). The Driver Hub STOP
   button always works.
6. Tuner results show on the Driver Hub and on Panels.

**Check the TeleOp sticks first.** Pedro's own teleop mapping is
`setTeleOpDrive(-left_stick_y, -left_stick_x, -right_stick_x, ...)`. A positive turn input turns the robot left
(counter-clockwise). `CompTeleOpControls.driveTurn` currently returns `+right_stick_x`, so the right stick will
probably turn the robot the wrong way. Fix: `return -driver.right_stick_x;`. (Strafe was already flipped.)

## 1. Constants.java setup

1. **Motor names and directions** are already `lf`, `lb`, `rf`, `rb` with left side REVERSE and right side
   FORWARD. This matches Pedro's defaults. Confirm in the Localization Test (section 2a): left stick forward
   drives forward, left stick sideways strafes, right stick turns.
   If the robot drives backwards or sideways wrongly, a motor direction or name is wrong (front motors swapped
   with rear motors is the usual cause).
2. **Mass:** weigh the robot **in kilograms** (pounds x 0.4536) and set `.mass(...)`. It is used for centripetal
   force correction. (If you stand on a scale holding it, subtract your own weight.)
3. Pods and IMU: the names, ports and wiring are in `HARDWARE_CONFIG.md`.

## 2. Localization (two-wheel + IMU)

### 2a. Localization Test (Tuning > Localization > Localization Test)
Drive the robot with the gamepad or push it by hand. Watch x, y and heading.

| You do | Must happen | If not |
| --- | --- | --- |
| Move the robot forward | **x increases** | Flip the forward pod: `.forwardEncoderDirection(Encoder.REVERSE)` |
| Move the robot left | **y increases** | Flip the strafe pod: `.strafeEncoderDirection(Encoder.REVERSE)` |
| Turn the robot left (counter-clockwise from above) | **heading increases** | Wrong Control Hub orientation (below) |
| Move forward only | y stays put | Pod names `rf`/`rb` swapped or wired to the wrong ports |

- Pedro's default for the forward pod is REVERSE and for the strafe pod is FORWARD. This project sets both
  FORWARD, so expect to flip at least one. That is normal.
- **Heading wrong or drifting:** `IMU_Orientation` in `Constants.java` must match how the Control Hub is mounted.
  `LogoFacingDirection` is the direction the REV logo on the hub faces on the robot. `UsbFacingDirection` is the
  direction the hub's USB port faces. Allowed values: UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT.
- The test starts the robot at pose (72, 72), the middle of the field.

### 2b. Offsets Tuner (Tuning > Localization > Offsets Tuner)
Finds `forwardPodY` and `strafePodX` automatically. It supports exactly two dead wheels, which is what you have.

**Requirement:** both offsets must be **0** when you run it. `Constants.java` already has `forwardPodY(0.0)` and
`strafePodX(0.0)`.

1. Place the robot in a corner of a field tile (or the field corner), facing a straight direction.
2. Select and START the Offsets Tuner.
3. Move the robot out of the corner, turn it **180 degrees**, and push it back into the **same corner**. A
   rectangular robot pressed into the same corner at 0 and 180 degrees ends with its center in the same place, so
   the only thing the localizer sees is the 180 degree turn.
4. The screen shows `strafeX` and `forwardY` in inches. Paste them:
   ```java
   .strafePodX(strafeX)
   .forwardPodY(forwardY)
   ```
5. Run it 3 times and average.
6. Check: spin the robot in place in the Localization Test. x and y must barely move.

Re-run this after the other subsystems are mounted, because the robot's center of rotation moves with its mass.
(You can also tape-measure the offsets; see `ODOMETRY.md`. Left of center is +y, forward of center is +x.)

### 2c. Forward Tuner (Tuning > Localization > Forward Tuner)
> **Known upstream bug, fixed in this project.** The Quickstart's `Tuning.java` for Pedro 2.1.2 starts the Forward and
> Lateral tuners at pose (72, 72) but divided by the absolute x or y, so "Distance Moved" read 72 too high and the
> multiplier came out far too small (it reads about 120 after a 48 in push instead of 48). Pedro had this right in
> commit `dbaffc0` and broke it again in `4c55679`. This project's `Tuning.java` subtracts the 72. If you ever replace
> the file with a fresh copy, re-apply that. **"Distance Moved" must read about 48 after a 48 in push.**

1. Mark 48 inches (2 tiles) on the floor in a straight line.
2. START it, then **push the robot forward exactly 48 inches** by hand.
3. Read **Multiplier**. That number is the value to use. Paste it:
   ```java
   .forwardTicksToInches(multiplier)
   ```
4. Run it 3 to 5 times and average. The goBILDA 4-bar pod starting guess in `Constants.java` is 0.001979.
5. Check: after updating, run again. "Distance Moved" should read about 48.

### 2d. Lateral Tuner (Tuning > Localization > Lateral Tuner)
Same as the Forward Tuner but **push the robot LEFT 48 inches** (the on-screen text says "right"; pushing left
gives a positive reading, which matches the official docs). Paste the multiplier:
```java
.strafeTicksToInches(multiplier)
```

### 2e. Turn Tuner: skip it
It is in the menu, but with the two-wheel localizer heading comes from the IMU and the turning multiplier is
fixed at 1 in the library. Pedro's two-wheel guide doesn't list it.

### 2f. Re-test
Run the Localization Test again. Drive a tile square and back: the pose should return close to where it started.
Do not go on until this is accurate.

## 3. Velocity tuners (Tuning > Automatic)

These measure how fast your robot goes at full power. Use a full battery and plenty of room.

**Forward Velocity Tuner:** the robot drives forward at full power until it has moved `DISTANCE` (default 48
inches), then cuts power. Larger distances give better results; you can raise `DISTANCE` in Panels. Read
**Forward Velocity** and paste it into `driveConstants`:
```java
.xVelocity(velocity)
```
**Lateral Velocity Tuner:** same, but the robot moves to the **LEFT**. Paste:
```java
.yVelocity(velocity)
```
Run each 3 to 5 times and average. Pressing **A** applies the number only until the robot is turned off.
Gamepad 1 **B** stops it.

## 4. Heading PIDF (Tuning > Manual > Heading Tuner)

The robot holds its heading. You turn it by hand and watch how it returns.

1. Open Panels > Configurables > Tuning > Follower > Constants and find `coefficientsHeadingPIDF`.
2. START the Heading Tuner. Turn the robot left or right by different amounts and let go.
3. Adjust P, I, D, F. After changing a value, press **Enter** in Panels to apply it.
   - Oscillates around the target: lower P (or add a little D).
   - Returns too slowly: raise P.
   - Avoid I. A small F (feedforward) helps overcome friction: set the other terms to 0 and raise F until the
     robot starts to twitch, then back off.
4. Paste the final numbers (P, I, D, F in that order):
   ```java
   .headingPIDFCoefficients(new PIDFCoefficients(P, I, D, F))
   ```

If the robot spins 180 degrees and oscillates, a motor is mapped to the wrong name (front and rear swapped) or the
pod offsets have the wrong sign.

## 5. Choose how the robot brakes and holds position

Pedro has two methods. **Use one, not both.**

### Option A: Predictive Braking (recommended to start)
Automatic, about 15% faster autos per the official docs, and you only hand-tune one number (kP). It works with
mecanum. The default is **off**; it turns on when you call `.predictiveBrakingCoefficients(...)`.
It replaces the translational and drive PIDFs, the centripetal tuner and the two zero-power-acceleration tuners.

1. **Tuning > Automatic > Predictive Braking Tuner.** Leave 4 to 5 feet in front of and behind the robot.
   It drives forward and backward 12 times (1 second each, from full power down to 0.2), braking with a small
   reverse power each time. **B** stops it. It's normal if the robot's heading turns or the robot lifts slightly
   while braking.
2. When it finishes it prints **kQuadraticFriction** and **kLinearBraking**. Paste them into
   `followerConstants`. **The argument order is (kP, linear, quadratic)** and the tuner prints quadratic first, so
   don't swap them:
   ```java
   .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(kP, kLinearBraking, kQuadraticFriction))
   ```
   Start with kP = 0.1. (Docs example: `(0.1, 0.04, 0.0016)`. Typical linear 0.05 to 0.15.)
   You need `import com.pedropathing.control.PredictiveBrakingCoefficients;`.
3. **Tuning > Tests > Line.** The robot drives 40 inches forward and back, looping. Raise kP as high as you can
   without jitter (typical 0.05 to 0.3). For smoother, earlier slowing try kP of 0.05 or less and a larger quadratic term.
4. Turn centripetal correction off: `.centripetalScaling(0)`.
5. Lower the path end constraint from 0.995 to **0.97 or 0.95** in `pathConstraints` (first argument). Do not go
   below 0.9 or the path ends before braking.
6. Use `PathChain`s wherever the robot doesn't need to fully stop. Predictive braking stops completely at the end
   of each separate path.

### Option B: PIDFs (more manual)
1. **Zero Power Acceleration tuners** (Tuning > Automatic): the robot accelerates to 30 in/s, cuts power and
   measures its deceleration. Needs about a tile of room. The lateral one moves **left**. Paste the (negative)
   numbers:
   ```java
   .forwardZeroPowerAcceleration(value)
   .lateralZeroPowerAcceleration(value)
   ```
2. **Translational Tuner** (Tuning > Manual): the robot holds position. Push it sideways and tune
   `coefficientsTranslationalPIDF` in Panels. Heading not correcting is expected here. Paste:
   `.translationalPIDFCoefficients(new PIDFCoefficients(P, I, D, F))`
3. **Drive Tuner** (Tuning > Manual): the robot drives 40 inches forward and back repeatedly (you can change the
   distance in Panels). It will drift sideways and turn because only the drive PIDF is on. Tune
   `coefficientsDrivePIDF`. Paste:
   `.drivePIDFCoefficients(new FilteredPIDFCoefficients(P, I, D, T, F))`
   Braking strength (`pathConstraints` 6th argument, default 1.25): raise it for stronger braking, lower it for a smoother stop.
4. **Centripetal Tuner** (Tuning > Manual): the robot drives a curve 20 inches forward and 20 left. If it
   corrects toward the inside of the curve, lower `centripetalScaling`. Toward the outside, raise it. Paste:
   `.centripetalScaling(value)`
   Mass must be set correctly first.

## 6. Validate (Tuning > Tests)

- **Line:** all controllers active, 40 inches forward and back. Should stop cleanly at both ends without
  oscillating.
- **Triangle:** straight lines and turns, looping.
- **Circle:** curved path while facing the center. If it drifts to one side, adjust centripetal scaling
  (PIDF option).

Then try a real path. `CompAuto` drives from (9, 72) to (36, 72) to (36, 36). Check the start pose matches where
you place the robot, and keep people clear.

## 7. When to re-tune

| Change | Re-run |
| --- | --- |
| Added/removed mechanisms or weight, new battery type | Velocity tuners, braking or PIDF tuners, Offsets Tuner, mass |
| Moved a pod | Offsets Tuner, ticks to inches |
| Changed wheels | Everything from section 2 |
| Different floor surface | Velocity and braking/PIDF tuners |

## Troubleshooting (from the official docs, with this project's specifics)

| Symptom | Likely cause |
| --- | --- |
| Pose goes the wrong way in the Localization Test | Flip that pod's `Encoder` direction; check `rf`/`rb` names |
| Heading wrong or drifting | `IMU_Orientation` doesn't match hub mounting |
| Robot never stops in a velocity tuner | Localization wrong. Redo section 2 |
| Robot spins 180 and oscillates in the Heading Tuner | Front and rear motors swapped, or offset sign wrong |
| Robot never moves in Translational/Heading tuners | Normal until you push it; if still nothing, P too low |
| Jittery movement | Lower `brakingStrength` (PIDF) or kP (predictive) |
| Panels Configurables empty | Select a tuner on the Driver Hub first, then reload the page |
| Robot drifts in the Drive Tuner | Expected: only the drive PIDF is on |
| Drives wrong in TeleOp but right in Localization Test | Check `CompTeleOpControls` stick signs |

Help: the Pedro Pathing Discord #tuning-help channel (include a video, your constants and telemetry).
