# Limelight 3A: seeing an opposing robot and driving round it (AVOIDANDPARK)

Goal: while the robot drives the Leave and Park route, the Limelight watches for an opposing robot. If one is in the way
the robot goes round it, or stops and waits if there is no room, and carries on when it is clear.
Read **How it works and its limits** first, then do the calibration in order. Each part ends with a check.

| File | What it does |
| --- | --- |
| `AvoidAndPark.java` | **Autonomous "AVOIDANDPARK"** (group Competition). The Leave and Park route plus the avoidance. |
| `AvoidRoute2.java` | **Autonomous "AVOIDROUTE2"** (group Competition). The second route (line, curve, line) plus the same avoidance. See **AVOIDROUTE2** below. |
| `AvoidOpModeBase.java` | The avoidance engine both autonomous OpModes share (seeing, planning, braking, safety checks). |
| `util/RoutePolyline.java` | A route leg as short straight pieces, so curves can be measured and rejoined. Unit tested. |
| `ObstacleVisionTest.java` | **TeleOp "Obstacle Vision Test"** (group Test). Never drives. Use it for the whole calibration. |
| `ObstacleSettings.java` | Every number: pipeline slots, distance constant, radii, margins, timing. Editable in Panels. |
| `util/ObstacleMath.java` | Distance from patch size, where the obstacle is on the field, and the detour planner. Unit tested. |
| `util/ObstacleTracker.java` | Remembers obstacles on the field map for a moment after the camera stops seeing them. Unit tested. |

## How it works and its limits

**What the camera looks for.** Every robot must carry alliance-coloured ROBOT SIGNS (manual rule R402; I could not read the
rule itself, so check its size, placement and colour wording). The camera runs a colour pipeline for the **opposing**
alliance's colour: red alliance looks for blue (slot 2), blue alliance for red (slot 1). `LeaveAndPark.ROTATE_180` chooses.

**Distance** comes from how big the coloured patch looks: `distance = DISTANCE_K / sqrt(area)`. The Limelight Java
interface gives angles and area, not the bottom edge of the patch, so this is the practical option. It is only as good as the
patch is the same size on every robot: expect about **plus or minus 30 percent**. Calibrate with the **smallest** sign you
expect (a bigger sign then looks nearer, which is the safe error).

**The field map.** Each sighting becomes a point on the field (the robot's odometry pose plus the bearing and distance). An
obstacle counts after it has been seen for 0.1 s and is forgotten 1.5 s after it was last seen. Sightings that land on the
HIVE or a flower are ignored, because they carry red and blue.

**What it does** (checked every 0.1 s):
- path clear: carry on;
- obstacle in the way and room beside it: a dog-leg round it, over to one side before the obstacle, alongside at a safe
  distance, back onto the route;
- no room beside it (the usual case for an obstacle in the middle of leg 2): the **long way round**, found by a grid search. For
  an obstacle in the middle of the corridor it goes south of the HIVE, up the east side, and back along the top, about 170 in
  longer, about 12 s in total at power 0.5. It is only taken if it fits in the time left in autonomous
  (`ALLOW_LONG_WAY_ROUND`, `LONG_WAY_MAX_EXTRA_IN`, `AUTO_LENGTH_SEC`). Switch it off to make the robot stop and wait instead;
- no way at all: stop and wait up to 4 s for it to move, then give up and stop;
- an obstacle suddenly at touching distance: stop at once.
All of these only go where the robot may legally stand: inside the walls and, when switched on, out of the HIVE and flower areas,
planned at the `PLAN_` sizes in `ObstacleSettings`. **Those four sizes are 0 now, so the HIVE and flower boundaries are OFF** and a
detour may go anywhere inside the walls, including through the middle of the field (restore with 25, 3, 4.5 and 0). Sightings on the
HIVE and flowers are still filtered out (`IGNORE_*`), because they carry red and blue. **Measure the HIVE frame and the flower towers before relying on the long way** (the defaults
are estimates read off a drawing, 10 in out is possible): it runs only a few inches outside the planned HIVE area.

**Not seeing is not gone.** An obstacle is only believed gone if the camera could see where it was and sees nothing there. A robot
stopped right beside an obstacle cannot see it (inside the blind zone), so the obstacle is kept for 8 s. Without this rule a
stopped robot concluded the obstacle had left and drove into it. A **flickering pipeline** is tolerated: sightings are matched by
bearing, not by the noisy distance estimate, and one seen within 60 in counts at once.

**Stopping** uses the motors' brakes. (Pedro's `breakFollowing()` lets them float, and a robot at 30 in per second then coasts
several inches, which is more than it has when an obstacle is first seen close.)

**Limits you must know**
1. **The corridor is narrow.** Leg 2 of the route runs between the wall and the HIVE, about 44 in wide. A robot needs about
   25 in of room to the side of an obstacle, so a short detour only exists for an obstacle hugging the wall. An obstacle in the
   middle of the corridor needs the long way round the HIVE (see What it does) or, with that switched off, stop and wait. In the
   simulation: wall-lane robot, short detour, finish with about 24 in to spare; middle-lane robot, the long way, finish at 12 s
   with 35 in to spare.
2. **The camera looks along the robot's heading.** On legs 1 and 3 of the supplied route the robot drives sideways to where it
   faces, so the camera cannot see where it is going. It reacts to what it saw earlier (1.5 s memory) or sees ahead. Leg 2 is
   the leg that works well.
3. **Blind zone.** Anything closer than a few inches from the lens (about 5 in for a sign 4 in below the lens with the camera
   tilted 25 degrees down; it depends on the tilt and the sign height) is not in the picture. An obstacle that appears that
   close cannot be seen in time.
4. **Tilt.** A camera tilted down for pollen loses a robot sign off the top of the picture at long range (25 degrees down:
   about 57 in for a sign 4 in below the lens). Part A checks this.
5. Only the opposing colour is seen: **an alliance partner is not**. Red or blue NECTAR on the floor, and tape, can fool the
   pipeline if its filters are loose (step 7).
6. If the obstacle pipeline is not confirmed, AVOIDANDPARK **drives the route without avoidance** (turn
   `ObstacleSettings.REQUIRE_VISION` on to make it refuse instead).
7. HIVE and flower positions are estimates read off a drawing (about 10 in out). Detours are planned around them, so measure them.

## If the robot drove straight through the obstacle

Two different things can cause this. Work down the list; the screen tells you which.

**1. The camera never saw it (the usual cause).** The AVOIDANDPARK screen shows, at INIT and during the run, how far detection
got. Put the obstacle in front of the camera at INIT:

| What the screen shows | What it means | Fix |
| --- | --- | --- |
| `!! AVOIDANCE IS OFF` | the pipeline slot could not be selected; with `REQUIRE_VISION` on (default) the robot will not move | check the Limelight cable, the config name `limelight`, and that the slot exists |
| `blobs in view` flickering between some and 0 | the pipeline is marginal: the sign is only caught in some frames. Detection still works, but it reacts later, so fix it: lower the exposure a little more, widen the hue and saturation range, loosen fullness and aspect ratio (Part B), make the sign bigger or better lit | Part B |
| `blobs in view 0` / `NO patch seen`, obstacle in front of the camera | the pipeline cannot see it: wrong **colour** for the pipeline, pipeline not built or not tuned, or the obstacle has no coloured sign | build and tune the pipeline (Part B). To test with a dummy of a different colour, set `ObstacleSettings.PIPELINE_OVERRIDE` to the slot whose colour matches the dummy |
| `patch: TOO SMALL` | the patch is under `MIN_AREA` | lower `MIN_AREA`; check the filters; use a bigger sign |
| `patch: IGNORED (lands on the HIVE / a flower)` | the estimated position falls in a HIVE or flower area, so it is thrown away | recalibrate `DISTANCE_K` (an over-estimated distance pushes the obstacle onto the HIVE); check the flower positions |
| `patch: KEPT` at INIT, but no reaction in the run | detection works; see the run line below | |

During the run the `vision` line counts the stages: `blobs N, big enough N, kept N, ignored N | tracks N, confirmed N | last
blob X s ago`. `blobs 0` all the way along the obstacle means the camera did not see it; blobs but `confirmed 0` means it was
seen for under 0.1 s or ignored.

Also check **where** the obstacle sat. The camera looks along the robot's heading: on legs 1 and 3 of the supplied route the robot
drives sideways to where it faces, so an obstacle there is not in the picture. Leg 2 (the long run up the field) is the one it can
watch. And check the colour: the red alliance looks for BLUE signs (slot 2), the blue alliance for RED (slot 1).

**2. It saw it but there was nowhere to go.** Then it should not drive through: it takes the long way round, or stops and waits
and gives up after `BLOCKED_WAIT_SEC` (long way off, or no time for it). The plan was blocked because the corridor beside the wall is only about 44 in wide (wall to the HIVE area).
Our robot is 16 in wide, so to pass a robot it needs a clear gap of at least 16 in plus a margin on one side. A dummy robot
18 in across in the **middle** of the corridor leaves at most 13 in on each side, which a 16 in robot cannot use. This is physics, not a
setting. What the planner decides, from the real code, for an obstacle on the route along leg 2:

| Obstacle centre, inches from the wall | defaults (room needed 25 in) | smaller dummy (radius 9, room needed 20 in) |
| --- | --- | --- |
| 4 to 6 | passes on the HIVE side | clear |
| 7 to 11 | passes on the HIVE side | passes on the HIVE side |
| 12 to 16 | **no room beside it: the long way round** | passes on the HIVE side |
| 17 to 40 | **no room beside it: the long way round** | **no room beside it: the long way round** |

A dummy anywhere else in the corridor cannot be passed locally (no setting makes a 16 in robot fit beside an 18 in robot in a 44 in
lane), so the robot takes the long way round the HIVE (`ALLOW_LONG_WAY_ROUND`), as in the last two rows. With the long way off, those
rows mean stop and wait. To test the short detour, put the dummy within about 10 in of the wall.

**3. It saw it late (the case that actually hit the dummy).** If the sign is only recognised from about 22 in away, the robot has
under 10 in to stop. The screen's `DISTANCE_K measured while approaching` line, shown during the run, checks the distance
constant against how the patch really grows as the robot closes in: a measured value far from the set one means every distance is
wrong by that factor (too large: the obstacle is seen as farther than it is and the robot reacts late).

## Calibration

### Part A: the camera mounting

1. The mounting is the one from `LIMELIGHT_POLLEN.md` (rigid, centred, forward, upright). The camera numbers are the ones in
   `PollenSettings`: `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_DEG`, `CAMERA_VERTICAL_FOV_DEG`. Finish the pollen calibration first.
2. **Measure how high a robot sign is** on the robots you will meet (the centre of the coloured patch, floor to centre, in).
   Enter the typical value as `ObstacleSettings.TYPICAL_SIGN_HEIGHT_IN`.
3. Run **Obstacle Vision Test** (Part C, step 9) and read the line **farthest a sign at that height stays in the picture**. It
   must be at least the range you want to react at: `TRIGGER_RANGE_IN` + the obstacle radius + our radius, 67 in with the
   defaults. If it warns, you have two ways out: tilt the camera up so the top of the picture reaches the horizon (about 20
   degrees or less, which also sees signs at any range) or lower `TRIGGER_RANGE_IN`. Tilting up has a price for the pollen
   chase: the closest pollen the camera can see gets farther away, so check `STOP_DISTANCE_IN` against it (pollen guide step 17).
4. Measure **`CAMERA_FORWARD_OFFSET_IN`**: how far in front of the robot's centre (the point the robot turns about, the one
   used for odometry) the lens is, along the direction the robot faces. Enter it in `ObstacleSettings`.

   Check: after steps 2 to 4 `PollenSettings` and `ObstacleSettings` hold your measured numbers, not the placeholders.

### Part B: teach the Limelight the robot-sign colours

Do this under the real field lights, with real robot signs. Two pipelines, one per colour, because the camera runs one at a time.

5. Limelight web interface (`limelight.local:5801`). Make a pipeline in **slot 1**, type Color/Retroreflective, name
   `red-robots`, and one in **slot 2**, `blue-robots`. (Slots 1 and 2 are `PIPELINE_RED_ROBOTS` and `PIPELINE_BLUE_ROBOTS`. Slot 0 is the
   pollen pipeline.)
6. For each: lower the exposure until the picture is slightly dark and the colour looks saturated; use the eyedropper on a sign
   of that colour; keep the hue range narrow; raise the minimum saturation and value so tape and tiles drop out.
7. **Contour filters.** Starting values to tune, not official ones: area from `MIN_AREA` (default 0.1 percent) upward, aspect
   ratio wide, about 1.2 to 6 (signs are rectangles, so this drops round NECTAR), **fullness 85 to 100** (a rectangle fills
   its box, a ball only about 78 percent), erode 1, dilate 2.
8. **False-positive sweep.** With the pipeline on, hold in front of the camera, on tiles, one at a time: red and blue NECTAR on
   the floor, red and blue tape, the HIVE faces, our own alliance's sign (must not show), a person in a red or blue shirt, a
   robot with no sign. Nothing but the opposing sign may be outlined. If something is, tighten the filters, or accept it only
   if it lands in the HIVE / flower ignore areas.

   Check: only an opposing robot's sign is outlined, in both pipelines.

### Part C: check it from the robot (no driving)

9. Build and deploy. Driver Hub: TeleOp > **Obstacle Vision Test**. At INIT it shows the Limelight status, `pipeline N selected`
   (N = slot 2 for the red alliance, slot 1 for the blue; set `PIPELINE_OVERRIDE` in Panels to test the other), the
   farthest-visible figure, and warns if the tilt is wrong. Press START.
10. Hold a sign in view. `patch 1` shows `tx`, `ty`, the area, an estimated distance, and its position **forward and left or
    right of the robot's centre**. The frame age should stay under 100 ms.
11. **Direction of tx.** Put the sign to the robot's right: the line must say **right**. If it says left, set
    `INVERT_TX = true` in `ObstacleSettings`.
12. **Calibrate DISTANCE_K.** Put the **smallest** sign you expect to meet at a measured distance in front of the lens, measured
    along the floor from the point under the lens, 24 in. In Panels set `ObstacleVisionTest.KNOWN_DISTANCE_IN` to 24 and read
    **suggested DISTANCE_K**. Repeat at 36 and 48 in and use the **average**. Put it in `ObstacleSettings.DISTANCE_K`, then set
    `KNOWN_DISTANCE_IN` back to 0. The reported distance should now be within about 15 percent at 24, 36 and 48 in. If it drifts
    far (area is not following the inverse square), the sign is tilted, glare is clipping it, or the filters are cutting it.
13. **Set MIN_AREA.** It decides how far away a robot is noticed. `MIN_AREA` = (`DISTANCE_K` / farthest distance you care about)
    squared. For `DISTANCE_K` 5 and 70 in that is 0.0051, a bit over 0.5 percent. The line `farthest it can see with MIN_AREA` at
    INIT shows the result. Keep it equal to or above the pipeline's area filter. Larger = ignores small false patches (NECTAR),
    smaller = notices robots earlier.
14. Walk a robot (or a board with a sign at the same height) across and toward the robot. `** IN THE WAY **` should appear
    when it is within the lane (about `room needed` inches to either side of the centreline) and inside the trigger range.
15. Panels changes are temporary. Copy the final numbers into `ObstacleSettings.java` and rebuild.

   Check: distances within about 15 percent at 24, 36 and 48 in; the right side reads right; nothing but a sign shows.

### Part D: the first avoidance runs

16. Open floor first if you can, otherwise the field. `LeaveAndPark.MAX_POWER = 0.3`, a hand on STOP. Put a board or a robot
    with a sign of the opposing colour as the obstacle, pushed by a person, never a real robot at first.
17. **No obstacle.** AVOIDANDPARK must drive the Leave and Park route exactly as before. Screen: `mode LEG`, `0 obstacle(s)`.
18. **Obstacle hugging the wall in leg 2** (about 10 in from the wall, 30 to 40 in ahead of the robot): the screen shows the obstacle
    position, `last plan DETOUR ... round the right`, the robot moves over, passes, returns and finishes. Measure the gap at the
    closest point: about 24 in centre to centre is the plan.
19. **Obstacle in the middle of the corridor:** the robot stops, the screen says `blocked, waiting`, and after 4 s `ABORTED`.
    Remove the obstacle after 2 s: it should resume.
20. Compare the position the screen gives for the obstacle with a tape measure. If it is consistently off, redo step 12
    (distance) or step 4 (camera offset). Raise the speed only when all of this works.

**Tuning**

| Symptom | Change |
| --- | --- |
| Stops for things that are not robots | tighten the pipeline filters (step 7); raise `MIN_AREA`; check `IGNORE_STATIC_ELEMENTS` |
| Does not notice a robot until it is close | lower `MIN_AREA`; check the farthest-visible warning (step 3); raise `TRIGGER_RANGE_IN` |
| Waits too long or gives up too soon | `BLOCKED_WAIT_SEC`; remember the route has to finish inside 30 s |
| Goes round too wide, or never finds room | `OBSTACLE_RADIUS_IN` (12), `ROBOT_CLEARANCE_RADIUS_IN` (10), `CLEARANCE_MARGIN_IN` (3): each inch costs room in a narrow corridor |
| Touches or nearly touches the obstacle | raise `OBSTACLE_RADIUS_IN` and `CLEARANCE_MARGIN_IN`; redo the distance calibration with a smaller sign |
| Flickers between going round and waiting | raise `BLOCK_SLACK_IN`; raise `CONFIRM_TIME_SEC` a little |
| Takes the long way when it should wait (or the reverse) | `ALLOW_LONG_WAY_ROUND`, `LONG_WAY_MAX_EXTRA_IN` |
| Forgets an obstacle it is still beside | raise `HARD_MEMORY_SEC` |
| Stops for something that is not there, then carries on | raise `MEMORY_SEC` slightly; tighten the pipeline filters |
| Detour goes into the HIVE or a flower | measure their positions; raise `PLAN_HIVE_*` / `PLAN_FLOWER_*` |

## AVOIDROUTE2: the second route with avoidance

`AvoidRoute2` drives the route exported from the Pedro visualizer (start `(56, 8, 90)`):

| Leg | Path | Heading |
| --- | --- | --- |
| 1 | line `(56, 8)` to `shoot1 (56, 25.3109)` | held at 90 |
| 2 | curve to `(11.9064, 107.7914)`, control point `(9.9074, 32.1442)` | 90 to 180 |
| 3 | line to `(56.1014, 116.2739)` | 180 to -90 (Pedro turns the short way, +90) |

`ROTATE_180` (the same switch as every other auto) turns the whole route for the blue side. "shoot1" is only a point where the robot
passes: there is no shooter code, so nothing is fired. The export's `PoseFactory` / `Paths.line(...).linear(...)` calls do not exist in
Pedro 2.1.2; the file rebuilds the same poses with `BezierLine` / `BezierCurve` and `setLinearHeadingInterpolation` /
`setConstantHeadingInterpolation`.

**Settings** (Panels, `AvoidRoute2`): `MAX_POWER` 0.5, `START_DELAY_SEC`, `HOLD_END`, `TIMEOUT_SEC` 25,
`MAX_OFF_PATH_IN` 8, `CURVE_LOOKAHEAD_IN` 12. The alliance switch, the field checks (`ENFORCE_*`) and every camera / avoidance number
are shared with the other autos (`LeaveAndPark`, `ObstacleSettings`), so the calibration above applies unchanged.

**On the curve** the robot follows the curve until something is in the way, goes round it, and **rejoins the curve a little past it**
(`REJOIN_BUFFER_IN`, 10 in); the rest of the curve is the same Bezier curve cut at that point, so the route keeps its shape. If the
obstacle is so near the end of the curve that there is no room to rejoin, it goes to the end of the leg. On the two lines it goes round
and on to the end of the leg, as AVOIDANDPARK does.

**First run, no obstacle.** Watch `off the path (in)` on the screen: the robot follows a curve less tightly than a line. Set
`MAX_OFF_PATH_IN` a few inches above the largest value you see (it stops the robot if it is further off than that).

**What the INIT screen warns about, and why it matters.**

- *CAMERA COVERAGE.* The camera is fixed to the robot and looks along its **heading**, not along the direction it travels. On this
  route the heading turns 90 to 180 degrees while the robot travels north-west and then north, so on most of the route an obstacle
  on the path is not in view until it is too close to stop for. The screen works this out: a point of the route counts as covered if,
  from some earlier pose at least 30 in before it, the camera could see it from no more than `TRIGGER_RANGE_IN` (45 in) away. With
  the settings as shipped about **31 percent** of the route is covered: **the middle of the curve, about `(20, 54)` to `(12, 91)`**.
  Not covered: the first part of the curve `(49, 27)` to `(20, 54)`, and the end of the curve and all of leg 3, `(12, 91)` to
  `(54, 116)`. The blue side is the same turned 180 degrees.
- *Simulation, sign recognised only within 45 in and flickering* (the closest the robot's centre got to a dummy robot's centre; touching is below
  about 17 in):

  | Dummy at | Result |
  | --- | --- |
  | `(18, 58)`, `(14, 74)` (covered) | went round, 24 in |
  | `(13, 88)` | stopped and waited, gave up after 4 s: the end of the curve is only 20 in from it, inside the 25 in clearance, so the robot cannot get there |
  | `(30, 38)`, `(12, 98)`, `(21, 109)`, `(32, 112)`, `(44, 114)` (not covered) | **drove into it** (under 1 in) |
  | `(22, 49)` | seen 18 in away, braked and went round, but brushed it (14 in) |

  Anything that makes the camera see further (a better pipeline, a larger sign) or look where the robot goes helps. The route's own
  headings are your decision: making the front follow the curve would fix the camera, but the exported headings have the robot
  turning 90 degrees at the start of the curve and again at the start of leg 3. A second camera is the other way.
- *"The route passes within X in of flower tower N."* The flower keep-outs are off (you switched them off), so this is only a warning; the
  estimated positions are +/- 10 in, so measure the real towers. As the positions are set now the curve passes about 1 in from tower
  2 and leg 3 about 0 in from tower 1 (the blue side: towers 4 and 3).

**Other behaviour to expect.**

- An obstacle within 25 in of a leg's END point blocks that leg: the robot cannot get there, waits `BLOCKED_WAIT_SEC` (4 s), and
  stops. This includes `shoot1`, 17 in from the start.
- A first sighting closer than about 20 in cannot be fully avoided: the robot brakes about 3 in after it sees the obstacle at full
  speed, so it may brush it. Lower `MAX_POWER` if that matters more than time.
- An obstacle moving across the path is planned for where it was seen, not where it is going.
- A replan of the detour the robot is already driving (same waypoints, within 8 in) no longer counts toward `MAX_DETOURS`, and an
  emergency stop does not re-fire every loop once the robot is on its detour (it re-arms when an obstacle is 5 in closer than at the
  last plan). This also applies to AVOIDANDPARK; in the simulation it turned the "more than 10 detours" aborts (sign recognised only
  within 18 to 22 in) into completed routes, and changed nothing else.
- `HOLD_END` now applies to the last leg of AVOIDANDPARK too (it used to apply only to a one-leg route), the same as Leave and Park.

**Checks to run (in this order, `MAX_POWER` 0.3 and a hand on STOP):**
1. No obstacle: it drives the whole route, ends facing -90 at `(56.1, 116.3)`, `off the path` stays under the limit.
2. A board with a sign on the covered part of the curve, about `(14, 74)`, 10 in from the wall side: it should be noticed and gone round.
3. A board on the first part of the curve, about `(30, 38)`, and across the top, about `(32, 112)`: expect it NOT to be noticed. This
   shows you the uncovered stretches on the real robot; keep your hand on STOP.
4. A board at `shoot1` `(56, 25)`: it should stop and wait, then give up after 4 s.
5. Repeat once with `ROTATE_180 = true` on the blue side.

## What is not checked from here

- The text of rule R402 (sign size, height, placement) and whether signs face forward on real robots.
- The Limelight web-interface wording, and the filter values (starting points only).
- That `ColorResult.getTargetArea()` and `LLResult.getStaleness()` exist under those names in SDK 11.0 (same as the pollen code).
- How the real distance error behaves across different teams' signs. The simulation uses a sign that is exactly the calibrated size.
- The camera's horizontal field of view (54.5 degrees, from memory) used for "could the camera see this?", and the sign height.
- How far the robot coasts and how quickly the brakes stop it. The simulation uses a 0.1 s brake time constant and a 0.45 s float one,
  which are estimates: check by stopping the robot at speed and measuring.
- Nothing here has been run on the robot.
