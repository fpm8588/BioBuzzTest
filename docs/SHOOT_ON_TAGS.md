# Shoot On Tags: drive to the shooting spot under the CELLs and shoot

Autonomous **Shoot On Tags** (group Competition) and its check **Shoot On Tags Test** (group Test). The autonomous does one thing: it works out
where the shooter has to stand to score into the CELLs above its alliance's AprilTag strip, drives there from the start, shoots, and stops.
It does not use the obstacle avoidance or the pollen chase.

| File | What it does |
| --- | --- |
| `ShootOnTags.java` | The autonomous. |
| `ShootOnTagsTest.java` | Never drives. Shows the plan, then compares it with what the camera sees. |
| `ShootSettings.java` | Every number: HIVE size, which face the strip is on, shooting distance, start, speeds, timings. Editable in Panels. |
| `ShootSetup.java` | Builds the plan and runs every check. Both OpModes use it, so the test shows exactly what the autonomous will do. |
| `util/ShootPlan.java` | The geometry (strip position, shooting pose, route round the HIVE). Unit tested. |

## What the plan is

The tags hang on the **underside of the baskets (CELLs)** of the HIVE, high up on its frame (red CELLs on the red half, blue on the blue half).
The robot is placed from the HIVE's size and position:

1. The HIVE frame is a rectangle, **about 49 in along x by 39 in along y, centred on the field centre (72, 72)** (so x 47.5 to 96.5, y 52.5 to 91.5). These are
   the estimates from the earlier field work (`LeaveAndPark`, `LeaveParkPlan`), not measurements.
2. The red strip is on one face of it, `ShootSettings.STRIP_FACE`, a distance `STRIP_ALONG_OFFSET_IN` along that face from its centre.
   **Default: the face toward the bottom wall (-y), in the middle of the red half** (x = 72 - 12.25 = 59.75, y = 52.5). The reasoning: the red robots start on the
   bottom wall in the red half (x below 72), and the red CELLs are on the red half. **This is my reading of the drawing, not something I could check.**
3. The robot stands `SHOOT_DISTANCE_IN` (camera to the strip's plane, default 30) straight out from that face, looking at the strip. With the camera 6 in
   ahead of the robot's centre that is the robot centre 36 in out: red **(59.75, 16.5), facing +y**.
4. The BLUE alliance's plan is the red one turned 180 degrees about the field centre (`LeaveAndPark.ROTATE_180`, like every other autonomous): strip on
   the top face at x = 84.25, shooting position **(84.25, 127.5), facing -y**.
5. The route from the start (red (56, 8) facing +y, the same start as the other autos) goes straight there if that is clear; otherwise round the HIVE frame plus
   `HIVE_ROUTE_MARGIN_IN` (6 in) using the grid search from the obstacle code. With the defaults it is a short straight line.

What `ShootPlan` refuses (the robot does not move, the screen says why): a standoff too small (the robot's front must be 2 in clear of the face) or too large
(the robot's back would go through the field wall; with the defaults the standoff may be 4 to 38.5 in), a strip off the end of its face, a shooting position that
is not legal, no route, or a start that breaks the set-up rules (`LeaveAndPark.ENFORCE_FIELD_CHECKS`).

## After arriving

1. The shooter has been spinning since the robot started driving (`USE_HIGH_SHOT` picks `Shooter.HIGH_VELOCITY` or `LOW_VELOCITY`); the adjustor is at `ADJUSTOR_POSITION`.
2. **Fine positioning (only if a tag of the strip is in view and `USE_VISION`)**: the robot turns to face the strip's centre and creeps until the camera's range is
   within `RANGE_TOLERANCE_IN` of `SHOOT_DISTANCE_IN`, never more than `MAX_CORRECTION_IN` (8 in) from the planned spot and for at most `VISION_TIMEOUT_SEC`. With no tag in
   view for `AprilTagSettings.AUTO_NO_TAG_SKIP_SEC` it skips this and shoots from the planned spot.
3. After `SPINUP_SEC` from the shooter starting, the intake feeds for `FEED_SEC`, then shooter and intake stop.

Safety while it runs: it stops if the robot touches a wall or the HIVE keep-out, is more than `MAX_OFF_PATH_IN` off the route, or takes longer than `TIMEOUT_SEC`.

## THE CAMERA PROBABLY CANNOT SEE THE TAGS

The camera is mounted for pollen: 3 in up, looking level, 42 degree lens (`PollenSettings`). The tags are high. Read off the HIVE drawing against its 49 in width, the
lower cluster is about **37 in** up and the upper about **51 in** (plus or minus 20 percent; `ShootSettings.TAG_HEIGHT_ESTIMATE_IN` = 37 until `AprilTagSettings.TAG_CENTRE_HEIGHT_IN` is set).
With a 37 in tag a level camera sees it only from **89 in or more** away; at the 30 in shooting distance the tag is 49 degrees above the lens axis and the picture ends at 21.
To see it from there the camera must be tilted **about 49 degrees up** (`PollenSettings.CAMERA_PITCH_DEG` = -49). That conflicts with looking at the floor for pollen, so a second
camera or a different mount may be needed. Until then the autonomous positions the robot by odometry alone (the INIT screen and the test say so), and the HIVE measurements below matter a lot.

## Setting it up on the field (in this order)

1. **Limelight:** the AprilTag pipeline is slot 2 (`AprilTagSettings.APRILTAG_PIPELINE` = 2, set). Family 36h11, tag size set in the pipeline; put the same size in inches in `AprilTagSettings.TAG_SIZE_IN`.
   (`ObstacleSettings.PIPELINE_BLUE_ROBOTS` is also 2, but is only used when `PIPELINE_OVERRIDE` is turned off.)
2. **Measure the HIVE** on the real field: the frame's x and y size and its centre, to the inch -> `HIVE_SIZE_X_IN`, `HIVE_SIZE_Y_IN`, `HIVE_CENTRE_*`.
   Which face the red strip is on and how far along it -> `STRIP_FACE`, `STRIP_ALONG_OFFSET_IN`. The height of its tags -> `AprilTagSettings.TAG_CENTRE_HEIGHT_IN`.
3. **Run Shoot On Tags Test.** INIT shows the plan: strip position, shooting position, route, checks. Compare each number with a tape measure.
4. **Put the robot by hand at the shooting position** the test shows (measure from the HIVE face and the walls) and press START. It shows whether the tags are in the camera's view, how far the camera
   says the strip is against the plan, and how far off-centre; if the numbers disagree, either the robot is not where you think or the HIVE numbers are off. It also prints the tag height from the
   Limelight's 3D pose, for `TAG_CENTRE_HEIGHT_IN`.
5. Tune `SHOOT_DISTANCE_IN` for your shooter (the test shows the allowed range), `USE_HIGH_SHOT`, `ADJUSTOR_POSITION`, `SPINUP_SEC`, `FEED_SEC`.
6. **Run Shoot On Tags** with the power low (`MAX_POWER` 0.3) and a hand on STOP, no game pieces at first. Then at 0.5.
7. Panels changes are temporary. Copy the final numbers into `ShootSettings.java` and rebuild.

## What was tested here

On a laptop, with a simulated robot (nothing has run on the real one): the plan for every face, both alliances, and the limits; the route round the HIVE when the strip is on the far face
(it goes via (32, 40) and (32, 106)); the whole autonomous start to finish (red, blue, far face, a refused run with no motor command, no Limelight, a Limelight that sees nothing); and the fine
positioning with a simulated camera when the strip is 4 in and 12 in farther or 4 in nearer than planned and the robot's heading is 5 degrees off (it turns to face the strip, moves only as far as needed, and never more than 8 in).
Not checked: the real HIVE and tag positions, the tag heights, the Limelight SDK names used for tags (same caveat as `LIMELIGHT_APRILTAG.md`), and the shooter's range.
