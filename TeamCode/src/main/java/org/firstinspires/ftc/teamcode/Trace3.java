package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.List;

/*
 * TRACE 3: the route from the Pedro Pathing export, driven like Leave and Park (see RouteAutoBase): just the route, no Limelight, no avoidance,
 * no intake or shooter. The numbers are the export's:
 *
 *   start   (56, 8, 90)
 *   path1   line   start -> (56.305, 34.5605, 90)                          heading held at 90 (.constant(path1))
 *   path2   line   (56.305, 34.5605) -> (20.8177, 35.3673, 90)             heading held at 90 (.constant(point2)): drives sideways to the left
 *   path3   line   (20.8177, 35.3673, 90) -> (20.2889, 117.6906, 0)        heading 90 -> 0
 *   path4   line   (20.2889, 117.6906) -> (9.6135, 117.4992, 0)            heading held at 0
 *
 * ROTATE_180 (Panels > LeaveAndPark, the same switch as every other autonomous) turns all of it 180 degrees about the field centre for blue.
 * See docs/TRACE_ROUTES.md for what the INIT checks say about this route.
 */
@Autonomous(name = "Trace 3", group = "Competition")
public class Trace3 extends RouteAutoBase {

    @Override
    protected String title() {
        return "TRACE 3";
    }

    @Override
    protected List<Leg> buildLegs() {
        Pose start = pose(56, 8, 90);
        Pose path1 = pose(56.305, 34.5605, 90);
        Pose point2 = pose(20.8177, 35.3673, 90);
        Pose point3Start = pose(20.8177, 35.3673, 90);
        Pose point3 = pose(20.2889, 117.6906, 0);
        Pose point4 = pose(9.6135, 117.4992, 0);

        return Arrays.asList(
                line(start, path1, true),                  // Paths.line(start, path1).constant(path1)
                line(path1, point2, true),                 // Paths.line(path1, point2).constant(point2)
                line(point3Start, point3, false),          // Paths.line(point3Start, point3).linear(point3Start, point3)
                line(point3, point4, true));               // Paths.line(point3, point4).constant(point4)
    }
}
