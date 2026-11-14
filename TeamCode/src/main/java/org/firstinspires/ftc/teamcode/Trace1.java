package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.List;

/*
 * TRACE 1: the route from the Pedro Pathing export, driven like Leave and Park (see RouteAutoBase): just the route, no Limelight, no avoidance,
 * no intake or shooter. The numbers are the export's:
 *
 *   start   (56, 8, 90)
 *   path1   line   start -> (56.1281, 19.7717, 90)                                          heading held at 90 (.constant(path1))
 *   path2   curve  (56.1281, 19.7717) -> (55.7527, 126.014), control point (0, 66.6114)       heading 90 -> -90
 *   path3   line   (55.7527, 126.014, -90) -> (6.9199, 118.1818, 0) in the export; here shortened to (9, 118.5154, 0)  heading -90 -> 0
 *           (same line, same headings, 2.1 in shorter, so the 16 in robot's back edge ends 1 in inside the wall instead of 1.08 in outside it)
 *
 * ROTATE_180 (Panels > LeaveAndPark, the same switch as every other autonomous) turns all of it 180 degrees about the field centre for blue.
 *
 * READ THE INIT SCREEN: path3 crosses x = 40 at about y = 123.5, 4.5 in from the estimated centre of flower tower 1 at (40, 128), so the
 * robot's body reaches the tower (a warning only: that position is good to +/- 10 in, so check it on the real field).
 */
@Autonomous(name = "Trace 1", group = "Competition")
public class Trace1 extends RouteAutoBase {

    @Override
    protected String title() {
        return "TRACE 1";
    }

    @Override
    protected List<Leg> buildLegs() {
        Pose start = pose(56, 8, 90);
        Pose path1 = pose(56.1281, 19.7717, 90);
        Pose point2Start = pose(56.1281, 19.7717, 90);
        Pose point2 = pose(55.7527, 126.014, -90);
        Pose point2Control1 = pose(0, 66.6114, 0);      // a control point: only its x and y are used
        Pose point3Start = pose(55.7527, 126.014, -90);
        Pose point3 = pose(9, 118.5154, 0);   // export: (6.9199, 118.1818); shortened along the same line so a 16 in robot's back edge is 1 in inside the wall

        return Arrays.asList(
                line(start, path1, true),                          // Paths.line(start, path1).constant(path1)
                curve(point2Start, point2Control1, point2, false),  // Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2)
                line(point3Start, point3, false));                 // Paths.line(point3Start, point3).linear(point3Start, point3)
    }
}
