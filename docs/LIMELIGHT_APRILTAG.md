# Limelight 3A: aiming at the goal AprilTag and measuring the range to it

Goal: the robot turns to face the centre of its alliance's scoring strip of AprilTags and knows how far away it is. Aiming works in TeleOp
(hold the driver's right bumper) and in the autonomous (a short aim before the shot). The range is shown on the screen now and is what a later step uses to pick
the shooter speed and the adjustor angle.

| File | What it does |
| --- | --- |
| `GoalTagTest.java` | **TeleOp "Goal Tag Test"** (group Test). Never drives. Use it for the whole set-up and check. |
| `AprilTagSettings.java` | Every number: pipeline slot, strip IDs, tag size, range method, aiming gains. Editable in Panels. |
| `subsystems/GoalTracker.java` | Finds the alliance's strip; gives turn command, aimed, and range. Usable from any OpMode. Works without a Limelight (aiming is just off). |
| `subsystems/Limelight.java` | `getTags()` added: every tag in the frame with ID, angles and 3D pose. |
| `util/TagMath.java` | The arithmetic (strip centre, range two ways, aim turn). Unit tested. |
| `CompTeleOpRed.java`, `CompTeleOpBlue.java` (on the Driver Hub: "Competition TeleOp RED" / "BLUE") | Identical TeleOps (all behaviour is in `CompTeleOpBase`) except for the strip: hold **driver right bumper** to aim at the red or blue strip. |
| `CompAuto.java` | The auto turns to the strip before it shoots; its alliance is `LeaveAndPark.ROTATE_180` like every autonomous. |

## The tags (from the printed strips)

All are family **36h11**. Each strip is four tags in a row, IDs left to right as you face it: two tags, a gap, two tags.

| Strip | IDs | Used for |
| --- | --- | --- |
| RED SCORING | 30 31 32 33 | aiming (red alliance) |
| RED AUDIENCE | 34 35 36 37 | not used yet |
| BLUE AUDIENCE | 38 39 40 41 | not used yet |
| BLUE SCORING | 42 43 44 45 | aiming (blue alliance) |

**The robot aims at the strip's centre, not at one tag.** The tags sit at known places along the strip (about -2.0, -0.85, +0.85, +2.0 tag widths
from the centre, measured off the pictures), so from whichever of the four the camera can see, it works out where the centre is and averages
the estimates. That needs the tag size: **set `AprilTagSettings.TAG_SIZE_IN`** (the black square, edge to edge, in inches = the pipeline's tag size in mm / 25.4). Without it the
robot aims at the middle of the tags it can see, which is up to two tag widths off-centre when only part of the strip is in view (about 10 degrees at 70 in with a 6.5 in tag).
It assumes the robot faces the strip squarely; 25 degrees off square adds about 1 degree of error.

## What you still have to measure

- **Printed size** of a tag: the black square only, edge to edge. In mm in the Limelight pipeline, and in inches in `AprilTagSettings.TAG_SIZE_IN`.
- **Height of the strip's tags above the floor** (centre of the black square), inches, into `AprilTagSettings.TAG_CENTRE_HEIGHT_IN`. GoalTagTest also works it out for you from the 3D pose.
- If the shooter is not under the strip's centre, the offsets in Part C step 6.

## Part A: the Limelight pipeline (laptop)

These steps are from my memory of the Limelight docs; menu names can differ a little in your firmware version.

1. Connect the Limelight to a laptop by USB-C or Ethernet and open `http://limelight.local:5801` (or `172.29.0.1:5801` over USB).
2. Pick an **empty pipeline slot** (not the pollen or robot-colour ones). The code assumes slot **3**; if you use another, change `AprilTagSettings.APRILTAG_PIPELINE`. Name it `apriltag`.
3. Set the pipeline type to the AprilTag one ("Fiducial Markers" / "AprilTag").
4. Set the **tag family** and the **tag size in mm** (the black square). A wrong size makes the 3D range wrong by the same percentage.
5. Optional: filter to only the goal tag IDs, so other tags cannot be picked.
6. Lower the **exposure** until the tag is crisp and the background dark (a moving robot blurs the tag otherwise).
7. Hold a printed tag 1 to 2 m away: the overlay shows the correct ID, steadily. Save.

## Part B: robot configuration

The Limelight must be in the active configuration, named exactly `limelight` (see `HARDWARE_CONFIG.md`). Without it Competition TeleOp and
Auto still run; aiming is simply off and the screen says "no Limelight in the configuration".

## Part C: check it with Goal Tag Test (nothing moves)

1. Run **Goal Tag Test**. INIT shows whether the AprilTag pipeline was confirmed and which goal ID it is looking for.
2. Hold the strip (or any of its tags) in view. Check: tag IDs are right, tags of the strip are marked `*`, `tracker` says `strip 30-33: n tag(s)`, and the frame age stays under 100 ms. Flip `LeaveAndPark.ROTATE_180` in Panels to test the blue strip.
3. **Which way it turns.** Move the strip to the right of the picture: `aim would` must say `turn RIGHT`. If it says the opposite, set `AprilTagSettings.INVERT_TX`.
4. **Range.** Put the camera a measured distance from the tag (along the floor, lens to the foot of the tag) and type that into `GoalTagTest.KNOWN_DISTANCE_IN`. The screen shows the error of each method:
   - *3D pose* needs the correct tag size in the pipeline (Part A step 4). Good to a few percent when it is.
   - *vertical angle* needs `TAG_CENTRE_HEIGHT_IN`, and the camera height and tilt in `PollenSettings`.
   Use the one that matches the tape measure (`AprilTagSettings.USE_POSE_DISTANCE`). Repeat at 3 or 4 distances over the range you shoot from.
5. Copy `tag centre height by 3D pose` into `TAG_CENTRE_HEIGHT_IN` once the pose range is right.
6. If the shooter is not in line with the camera, find `AIM_OFFSET_DEG` (degrees the crosshair should be to the right of the tag when a shot goes in) and `RANGE_OFFSET_IN` (camera to shooter).
7. Panels changes are temporary. Copy the final numbers into `AprilTagSettings.java` and rebuild.

## Part D: on the robot

- **TeleOp.** Pick **Competition TeleOp RED** or **BLUE** on the Driver Hub. Hold the driver's right bumper: the robot turns to the strip while the driver still drives. The screen shows `[AIMING]` then `[AIMED]`. If no tag of the strip is in view the driver keeps the turn.
- **Autonomous.** At the score pose the shooter spins up while the robot turns to the strip, then the intake feeds. If no tag of the strip is in view after `AUTO_NO_TAG_SKIP_SEC` (0.4 s) it skips the aim; it never aims longer than `AUTO_AIM_TIMEOUT_SEC` (1.5 s).
- **From your own OpMode:** `GoalTracker goal = GoalTracker.create(hardwareMap, AprilTagSettings.RED_STRIP_FIRST_ID);` (or `BLUE_...`; without the second argument it follows `LeaveAndPark.ROTATE_180`), then each loop `goal.update();` and read `hasGoal()`, `getAimTurn()` (Pedro's turn sign, for `setTeleOpDrive`), `isAimed()`, `getDistanceIn()`.

**Tuning**

| Symptom | Change |
| --- | --- |
| Turns the wrong way | `INVERT_TX` |
| Swings past the tag and back | lower `K_TURN` or `MAX_TURN` |
| Stops short, never quite faces it | lower `AIM_TOLERANCE_DEG`, raise `MIN_TURN` |
| Aims off to one side when only part of the strip is in view | set `TAG_SIZE_IN` (the screen says `NOT centred` until it is) |
| Aims at the wrong strip | check which TeleOp you picked (RED / BLUE) and `ROTATE_180` for the auto |
| Range reads high or low by a constant percentage | tag size in the pipeline (3D pose) or `TAG_CENTRE_HEIGHT_IN` / camera height and tilt (vertical angle) |
| Loses the tag while turning | lower exposure; lower `MAX_TURN` |

## Limits you must know

- **The camera has to be able to see the tag.** The camera is on the pollen mount (`PollenSettings`: 3 in above the floor, looking level, 42 degree lens, which was set for finding pollen on the floor). A strip high on the goal is above the top of that picture when the robot is close. For example, a tag centre 29.5 in up (an example number, not this season's) is out of view closer than about **69 in**. GoalTagTest prints `closest the tag stays in view` once `TAG_CENTRE_HEIGHT_IN` is set; if that is farther than you shoot from, the camera has to be raised or tilted up (`PollenSettings.CAMERA_PITCH_DEG`, negative = up) for the goal. That conflicts with looking at the floor for pollen: two cameras or a different mount may be needed.
- The audience strips (34 to 41) are not used. They could be used later to correct the robot's position on the field.
- Aiming only turns the robot. It does not drive to a distance.
- The Limelight frame is up to 100 ms old and the robot keeps turning meanwhile, so a fast turn overshoots a little. Hence the low gains.
- Not checked here: the Limelight web-interface wording; that `LLResultTypes.FiducialResult` has `getFiducialId()`, `getTargetXDegrees()`, `getTargetYDegrees()`, `getTargetArea()` and `getTargetPoseCameraSpace()` under those names in SDK 11.0, and that its pose is in the camera frame used here (x right, y down, z forward, metres). Nothing has been run on the robot.
