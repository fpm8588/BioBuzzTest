# Limelight 3A: finding and chasing POLLEN

Goal: the Limelight finds a yellow POLLEN, the robot turns to face it and drives up to it (`Pollen Chase`).
AprilTags come later. Read this top to bottom the first time; each part ends with a check.

What is in the code:

| File | What it does |
| --- | --- |
| `subsystems/Limelight.java` | Talks to the camera. `getPollen()` returns the pollen to go for, or `null` (no result, stale frame, or too small). |
| `util/PollenMath.java` | Distance from the camera angle, picking which blob, and the turn / drive controller. Unit tested. |
| `PollenSettings.java` | Every number in one place: pipeline slot, camera height and pitch, gains, limits. Editable in Panels. |
| `PollenVisionTest.java` | **TeleOp "Pollen Vision Test"** (group Test). Never drives. Use it to set the camera up. |
| `PollenChase.java` | **Autonomous "Pollen Chase"** (group Competition). Search, turn, drive up to the pollen, stop. |
| `PollenChaser.java` | The same chase as a part another OpMode can start. Used by DRIVEANDGRAB. |
| `DriveAndGrab.java` | **Autonomous "DRIVEANDGRAB"** (group Competition). The Leave and Park route, then the chase. |

Sources for the Limelight side: the Limelight 3A quick-start and FTC programming guide, the Tennessee Robotics Center
BIOBUZZ Limelight guide, and the FTC SDK sample `SensorLimelight3A` in this project. The Limelight web interface labels
may differ slightly from what is written here, because the Limelight docs were not readable from the build machine.

## Part A: the camera on the robot

1. **Mount it rigidly.** Bolt it to the frame, not to an arm or turret that moves. Put it at the front, centred left to
   right, facing forward, upright. A camera that wobbles or shifts invalidates every number below.
2. **Tilt it down** so a pollen lying on the floor shows from about 6 in to about 4 ft in front of the robot. The more
   it tilts down, the closer it can see.
3. **Measure and write down** the height of the lens above the floor in inches. This is `CAMERA_HEIGHT_IN`. Do not trust
   the default of 10.
4. **Cable:** USB-C to USB-A, into the Control Hub's blue **USB 3.0** port. Strain-relieve the cable so it cannot pull out.
5. **Driver Hub:** three dots > Configure Robot > open your configuration > scan for the device. The Limelight shows up as
   an Ethernet/USB device. Tap its name and rename it exactly **`limelight`** (case matters), save, activate.
   (The SDK sample warns: do not confuse the address the Hub gave the Limelight with the Limelight's own web address.)

   Check: the configuration lists `limelight` next to the drive motors and `imu`.

## Part B: teach the Limelight what a pollen looks like

Do this with a **real yellow pollen on the real field tiles, under the real lights** you will compete in. A pipeline tuned
in a different room usually fails on the field.

6. Plug the Limelight into a laptop and open `http://limelight.local:5801` (or use Limelight's Hardware Manager).
7. Make a pipeline in **slot 0** (that is `POLLEN_PIPELINE`): type **Color / Retroreflective**, name it `pollen-yellow`.
8. **Input:** lower the exposure until the picture looks a little dark and the yellow looks saturated. That cuts glare and
   motion blur. Raise the black level slightly if the tiles look too bright.
9. **Thresholding (HSV):** use the eyedropper on a pollen in the stream, then fine-tune:
   - Hue matters most: keep the range narrow around yellow.
   - Raise the minimum **value** so dark pixels drop out, and the minimum **saturation** so pale tile and tape drop out.
   - The Add and Subtract wands widen or narrow the range by clicking pixels. Subtract the tiles, the tape, and any
     yellow on the field (the HIVE frame has yellow on it).
   - You want only the pollen white in the threshold view.
10. **Contour filtering**, starting values from the Tennessee Robotics Center guide (a third-party source, tune from here):
    erode 1, dilate 2, **area 0.05 % to 40 %** of the image, fullness 50 to 100 %, aspect ratio 0.6 to 1.6. The minimum area
    must match `MIN_AREA` in `PollenSettings` (0.0005 = 0.05 %).
11. **Check in the stream**, moving the pollen around:
    - one outline on a pollen, anywhere in view, from 6 in to about 4 ft;
    - nothing on the tiles, tape, HIVE, flowers, a **red or blue NECTAR** (it must be ignored), or other robots;
    - two pollen at once give two outlines;
    - it still works near the brightest lights and in the darkest corner.
    If it picks up the wrong thing, tighten hue / saturation / value or the area and shape filters.

    Check: only pollen is outlined. The pipeline is stored on the Limelight, so it survives power-off.

## Part C: check it from the robot (no driving)

12. Build and deploy. Driver Hub: TeleOp > **Pollen Vision Test**. At INIT it prints the Limelight name, temperature,
    frame rate and running pipeline, and "pipeline 0 selected". If it says not confirmed, check the cable and the config name.
13. Press START and hold a pollen in front of the camera. You should see `pollen FOUND`, `tx`, `ty`, `area` and a
    distance. With nothing in view it says `NONE`. The frame age should stay under 100 ms.
14. **Check the direction of tx.** Put the pollen to the robot's *right*: tx must be **positive** and the screen must say
    `turn RIGHT`. If it says the opposite, set `INVERT_TX = true` in `PollenSettings`.
15. **Set the camera height:** `CAMERA_HEIGHT_IN` in Panels > Configurables > PollenSettings = your step 3 measurement.
16. **Calibrate the pitch.** Put one pollen on the floor at a measured distance in front of the lens, measured along the floor
    from the point under the lens to the pollen's centre (try 24 in). In Panels > PollenVisionTest set
    `KNOWN_DISTANCE_IN` to that distance. Read **suggested CAMERA_PITCH_DEG** and enter it as `CAMERA_PITCH_DEG`. Repeat at
    12 in and 36 in. The suggestions should agree to about a degree; if not, the camera is moving or the height is wrong.
    Set `KNOWN_DISTANCE_IN` back to 0.

    Check: with the pitch set, the `distance from camera` reading matches a tape measure within 1 to 2 in at 12, 24 and 36 in.
17. Read `nearest pollen the camera can see`. `STOP_DISTANCE_IN` (default 10 in) must be at least 1 in above it, or the
    pollen drops out of the picture before the robot stops. Raise `STOP_DISTANCE_IN` or tilt the camera down more.
    (`CAMERA_VERTICAL_FOV_DEG` = 42 is the Limelight 3A figure from memory; check it against the spec sheet.)
18. Panels changes are temporary. Copy the final numbers into `PollenSettings.java` and rebuild.

## Part D: the first chase

19. Open floor, at least 20 in clear all round and 3 ft ahead; only one pollen out, 24 to 36 in in front. The defaults are
    the fast settings (`CHASE_MAX_POWER = 0.9`); for the very first drive on a new robot lower it to 0.3, then raise it again. The robot spins in place while searching, so it must not start against a wall.
20. Set where the robot is: `START_X`, `START_Y`, `START_HEADING_DEG` in `PollenSettings` (default 56, 20, facing +y).
    With **no field** (bench or empty floor) set `ENABLE_FIELD_GUARD = false`; otherwise the guard uses the assumed wall,
    HIVE and flower positions from Leave and Park.
21. Driver Hub: Autonomous > **Pollen Chase**. INIT shows the camera preview and the start pose. Press START with a hand
    ready on STOP.
22. Expected: it spins until it has seen a pollen for 0.1 s, turns to face it, drives up, slows, and stops at
    `STOP_DISTANCE_IN`. Screen shows `ARRIVED`.

**Tuning**

| Symptom | Change |
| --- | --- |
| Turns the wrong way | `INVERT_TX` (and re-check step 14) |
| Wobbles left and right while approaching | lower `K_TURN` |
| Turns too slowly, or stalls just short of aimed | raise `K_TURN` or `MIN_TURN` |
| Stops too far / too close | `STOP_DISTANCE_IN` |
| Too slow overall | `CHASE_MAX_POWER` first (it caps everything, 0.3 is a crawl), then `MAX_FORWARD`, `K_FORWARD`, `MAX_TURN` |
| Rushes in and overshoots | raise `LATENCY_LEAD_SEC` (try 0.2), or lower `MAX_FORWARD` |
| Slows down too early and creeps the last inches | lower `LATENCY_LEAD_SEC` or raise `MIN_FORWARD` |
| Never starts moving forward | raise `ALIGN_WINDOW_DEG` or `MIN_FORWARD` |
| Spins forever, never finds it | raise `SEARCH_TURN_POWER` a little, or check the camera sees it in Pollen Vision Test |
| Chases the wrong thing | go back to Part B step 11 |

If anything goes wrong the motors stop and the screen says why: the field guard, the 70 in travel limit
(`MAX_TRAVEL_IN`), the 20 s run limit (`RUN_TIMEOUT_SEC`), or an unconfirmed pipeline.

## DRIVEANDGRAB: the route, then the chase

Autonomous **DRIVEANDGRAB** drives the Leave and Park route start to finish (same route, checks and settings:
`LeaveAndPark.USE_SUPPLIED_ROUTE`, `ROTATE_180`, `MAX_POWER`, `ENFORCE_*`), stands still for `PAUSE_BEFORE_CHASE_SEC`, then runs
the pollen chase from where the route ended. The chase starts by searching toward the middle of the field, since the route
ends against a wall. It needs everything Pollen Chase needs: finish Parts A to C first.

- If the Limelight pipeline is not confirmed at INIT, the route still runs and the chase is skipped.
- LEAVE and PARK are judged at the end of autonomous. Chasing a pollen drives the robot out of the loading zone, so the route's
  points are lost unless it is back in the zone at 30 s. It is a test of the two parts together.
- During the chase the field guard stops the robot at any wall, HIVE or flower keep-out. Near the route's finish the
  keep-out around flower 1 and the HIVE can stop a chase toward pollen close to them. For a test with no field, or
  to chase anywhere, set `PollenSettings.ENABLE_FIELD_GUARD = false` (the route's own checks are separate).

## What is not checked from here

- The Limelight web interface wording and the contour numbers above (third-party starting values).
- The Limelight 3A vertical field of view (only used for the warning in step 17).
- That `ColorResult.getTargetArea()` and `LLResult.getStaleness()` exist under those names in SDK 11.0. They come from
  the Limelight docs and a BIOBUZZ vision repo. The Android Studio build will say at once if either is wrong.
- Nothing here has been run on the robot.
