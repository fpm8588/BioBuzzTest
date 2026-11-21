package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.List;

/*
 * TRACE 2: the route from the Pedro Pathing export, driven like Leave and Park (see RouteAutoBase): just the route, no Limelight, no avoidance,
 * no intake or shooter. The numbers are the export's:
 *
 *   start   (57.9008, 8.0085, 90)
 *   path1   line   start -> (60.4355, 23.7604, 90)                                heading held at 90 (.constant(path1))
 *   path2   line   (60.4355, 23.7604, 90) -> (60.7828, 119.8323, -90)             heading 90 -> -90 (a half turn: Pedro picks the direction)
 *   path3   line   (60.7828, 119.8323, -90) -> (9.511, 119.3137, 0)               heading -90 -> 0
 *
 * ROTATE_180 (Panels > LeaveAndPark, the same switch as every other autonomous) turns all of it 180 degrees about the field centre for blue.
 * See docs/TRACE_ROUTES.md for what the INIT checks say about this route.
 */
@Autonomous(name = "Trace 2", group = "Competition")
public class Trace2 extends RouteAutoBase {

    @Override
    protected String title() {
        return "TRACE 2";
    }

    @Override
    protected List<Leg> buildLegs() {
        Pose start = pose(57.9008, 8.0085, 90);
        Pose path1 = pose(60.4355, 23.7604, 90);
        Pose point2Start = pose(60.4355, 23.7604, 90);
        Pose point2 = pose(60.7828, 119.8323, -90);
        Pose point3Start = pose(60.7828, 119.8323, -90);
        Pose point3 = pose(9.511, 119.3137, 0);

        return Arrays.asList(
                line(start, path1, true),                          // Paths.line(start, path1).constant(path1)
                line(point2Start, point2, false),                  // Paths.line(point2Start, point2).linear(point2Start, point2)
                line(point3Start, point3, false));                 // Paths.line(point3Start, point3).linear(point3Start, point3)
    }
}
