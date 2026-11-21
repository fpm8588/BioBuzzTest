package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.List;

/*
 * TRACE 4: waits 10 s, shoots, then drives a short route. The route is from the Pedro Pathing export, driven like Leave and Park (see
 * RouteAutoBase). In order:
 *
 *   1. START: wait DELAY_SEC (10 s) with the robot still. (This is the first thing after START; LeaveAndPark.START_DELAY_SEC, which is 0, then
 *      applies on top of it, as for every auto.)
 *   2. The shoot step, the same as Trace 3 Shoot (ShootSequence): shooter to 2100 RPM, the intake for 6 s once the shooter is up to speed (and
 *      at least 3 s have passed), then both off.
 *   3. The route, with the numbers from the export except the start y (see below):
 *        start   (59.5915, 136, -90)                                       the export had y = 132.6591
 *        path1   line   start -> (59.6372, 117.1729, 0)                    heading -90 -> 0 (turns the short way: +90)
 *        path2   line   (59.6372, 117.1729) -> (10.6668, 122.3354, 0)      heading held at 0 (.constant(point2)): drives to the left
 *
 * START y: the export's 132.6591 puts a 16 in robot facing -90 (back to the top wall) 3.3 in away from the wall, and the start rules say the robot
 * must START TOUCHING the perimeter wall, so the INIT check refused the route. The start is y = 136, which puts the robot's back edge at y = 144.
 * Put the robot there (back against the top wall, x about 59.6, facing down the field). The rest of the route is unchanged, so leg 1 is 3.3 in
 * longer than in the export.
 *
 * ROTATE_180 (Panels > LeaveAndPark, the same switch as every other autonomous) turns the whole route 180 degrees about the field centre for
 * blue. See docs/TRACE_ROUTES.md for what the INIT checks say about this route.
 *
 * Time: 10 s + up to 11 s of shoot step = 21 s of the 30 s autonomous before the route starts: the route has about 9 s. Needs the motors "shoot"
 * and "intake" in the configuration.
 */
@Configurable
@Autonomous(name = "Trace 4", group = "Competition")
public class Trace4 extends RouteAutoBase {

    /** How long to wait after START before anything moves or spins, seconds. */
    public static double DELAY_SEC = 10.0;

    private ShootSequence shootSequence;

    @Override
    protected String title() {
        return "TRACE 4";
    }

    @Override
    protected List<Leg> buildLegs() {
        Pose start = pose(59.5915, 136.0, -90);   // export: y = 132.6591 (3.3 in off the wall: refused); 136 = the back edge on the wall
        Pose point1 = pose(59.6372, 117.1729, 0);
        Pose point2 = pose(10.6668, 122.3354, 0);

        return Arrays.asList(
                line(start, point1, false),     // Paths.line(start, point1).linear(start, point1)
                line(point1, point2, true));    // Paths.line(point1, point2).constant(point2)
    }

    @Override
    protected void onInit() {
        shootSequence = new ShootSequence(this);
    }

    @Override
    protected void initTelemetry() {
        telemetry.addLine(String.format("First: wait %.0f s after START, then the shoot step, then the route.", DELAY_SEC));
        shootSequence.initTelemetry();
    }

    @Override
    protected void beforeRoute() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DELAY_SEC) {
            telemetry.addData("state", "waiting to start: %.1f s left", Math.max(0, DELAY_SEC - timer.seconds()));
            telemetry.update();
            sleep(20);
        }
        if (!opModeIsActive()) return; // STOP was pressed during the wait
        shootSequence.run();
    }

    @Override
    protected void onFinish() {
        shootSequence.stop();
    }
}
