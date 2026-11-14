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
| 3 | line | (55.7527, 126.014) | (6.9199, 118.1818) | -90 to 0 |

### Two findings (checked in simulation, not on a robot)

* **Wall.** The route ends at x = 6.92 facing 0 degrees, so the robot's back edge is at 6.92 - L/2. With the 16 in robot size in
  `LeaveAndPark` that is -1.08 in: outside the field, so the INIT check **refuses** it (leg 3). A 14 in robot passes (-0.08, inside the
  0.5 in tolerance). Fixes: set the real robot length in `LeaveAndPark.ROBOT_LENGTH_IN` / `ROBOT_WIDTH_IN` if it is under about 14.8 in;
  or change the end x to 8 or more (about 11.4 also earns LEAVE in the loading zone, like the old route); or set
  `LeaveAndPark.ENFORCE_FIELD_CHECKS = false`, which turns off BOTH this check and the live wall stop (the robot may then drive into the wall).
* **Flower tower 1.** Leg 3 passes x = 40 at about y = 123.5; the tower is estimated at (40, 128), so the robot's body reaches it
  (warning only; the estimate is good to +/- 10 in). Look at the real field before a match.

### Simulation results (stand-in follower, real route and check code)

| Case | Result |
|---|---|
| Red, 16 in robot, checks on | refused at INIT, robot never moves |
| Red, 14 in robot | runs 3 legs, ends (6.92, 118.18) facing 0 |
| Blue (ROTATE_180), 14 in robot | ends (137.08, 25.82) facing 180 |
| Red, 16 in robot, checks off | drives it, ends (6.92, 118.18) |

Real-robot behaviour (how closely the curve is followed, the "off the path" value) still has to be seen on the field.

## Adding Trace 2 / 3

Copy `Trace1.java`, rename the class and `@Autonomous(name = ...)`, change `title()` and the poses, and list the legs with
`line(from, to, constantHeading)` and `curve(from, control, to, constantHeading)`. Override `endsInZone()` to return `false` if the route is not
meant to finish in the loading zone (then no LEAVE / PARK lines are shown).
