# TRACE 1, 2, 3: plain drive routes

Each Trace is one Driver Hub autonomous (group "Competition", names "Trace 1", "Trace 2", "Trace 3"). It drives the route from its Pedro
Pathing export, like Leave and Park: **no Limelight, no avoidance, no intake or shooter** (`AuxSettings.ENABLED` stays `false`).
The shared code is `RouteAutoBase`; a Trace class only lists its legs.

## What a Trace does

1. INIT: builds the route and runs the same checks as Leave and Park (start rules, field walls along the WHOLE route including curves, flower
   proximity). The result is on the Driver Hub / Panels. A failed check **refuses to run** after START (the robot does not move).
2. START: follows each leg in order (power `LeaveAndPark.MAX_POWER`, timeout `LeaveAndPark.TIMEOUT_SEC`), holding the final pose.
3. Live safety: stops if the robot gets outside the walls, or is further than `RouteAutoBase.Settings.MAX_OFF_CURVE_IN` (curves, 8 in)
   or `LeaveAndPark.MAX_OFF_LINE_IN` (straight lines, 6 in) from the path it should be on. "off the path" is on the screen: drive the route
   once and set the limit a few inches above the largest value you see.
4. Blue: set `LeaveAndPark.ROTATE_180` (Panels), the same switch as every other auto. The route turns 180 degrees about the field centre.

## Trace 1

| Leg | Shape | From | To | Heading |
|---|---|---|---|---|
| 1 | line | (56, 8) | (56.1281, 19.7717) | held at 90 |
| 2 | curve, control (0, 66.6114) | (56.1281, 19.7717) | (55.7527, 126.014) | 90 to -90 |
| 3 | line | (55.7527, 126.014) | (9, 118.5154) (export: (6.9199, 118.1818)) | -90 to 0 |

### Changes from the export, and one warning

* **Leg 3 shortened.** The export ended at x = 6.92, which puts a 16 in robot's back edge 1.08 in outside the field wall, so the INIT check
  refused it. The end point is now (9, 118.5154): the same line and the same headings, 2.1 in shorter, back edge 1 in inside the wall.
  (If you use a different robot length, the end x must be at least L/2 - 0.5.)
* **Flower tower 1 (warning only).** Leg 3 crosses x = 40 at about y = 123.5; the tower is estimated at (40, 128), so the robot's body reaches it
  (the estimate is good to +/- 10 in). Look at the real field before a match.

### Simulation results (stand-in follower, real route and check code)

| Case | Result |
|---|---|
| Red, 16 in robot, checks on | passes the INIT checks, runs 3 legs, ends (9, 118.52) facing 0 |
| Blue (ROTATE_180), 16 in robot | ends (135, 25.48) facing 180 |
| The export's original end point, 16 in robot | refused at INIT (wall), robot never moves |

Real-robot behaviour (how closely the curve is followed, the "off the path" value) still has to be seen on the field.

## Adding Trace 2 / 3

Copy `Trace1.java`, rename the class and `@Autonomous(name = ...)`, change `title()` and the poses, and list the legs with
`line(from, to, constantHeading)` and `curve(from, control, to, constantHeading)`. Override `endsInZone()` to return `false` if the route is not
meant to finish in the loading zone (then no LEAVE / PARK lines are shown).
