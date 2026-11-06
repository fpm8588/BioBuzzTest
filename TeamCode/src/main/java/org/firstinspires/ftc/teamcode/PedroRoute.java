package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;

import java.util.Arrays;

/*
 * Real Pedro Pathing route on the 144 x 144 inch field: (0,0) is the bottom-left corner, +x is toward the
 * right wall, +y is toward the far wall, heading is counter-clockwise from +x.
 *
 * Route (all legs run back to back with no stop between them):
 *   1. Straight line from the start to WAYPOINT_1, turning smoothly to WAYPOINT_1's heading on the way.
 *   2. Curve from WAYPOINT_1 to WAYPOINT_2, bent toward the CURVE_CONTROL point, heading follows the curve.
 *   3. Straight line from WAYPOINT_2 to the PARK pose, turning to the park heading on the way.
 * The robot only stops, and holds position, at the end of the whole route.
 *
 * Every number is a Configurable: change them in Panels (Configurables > PedroRoute) after INIT, or here. They are
 * read at START, so values edited during INIT are used. The poses are placeholders; set them for your real field
 * (the Pedro visualizer at https://visualizer.pedropathing.com shows the route before you drive it).
 *
 * Setup: put the robot on START_X / START_Y facing START_HEADING_DEG, then INIT and START.
 * The first run should be done with MAX_POWER low (0.4) and the robot on blocks or in open space.
 */
@Configurable
@Autonomous(name = "Pedro Route", group = "Competition")
public class PedroRoute extends LinearOpMode {

    // Where the robot is placed (inches, degrees).
    public static double START_X = 9.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEG = 0.0;

    // Leg 1: straight line to here.
    public static double WAYPOINT_1_X = 36.0;
    public static double WAYPOINT_1_Y = 72.0;
    public static double WAYPOINT_1_HEADING_DEG = 0.0;

    // Leg 2: curve to here. CURVE_CONTROL pulls the path toward it without the robot having to go through it.
    public static double CURVE_CONTROL_X = 60.0;
    public static double CURVE_CONTROL_Y = 100.0;
    public static double WAYPOINT_2_X = 60.0;
    public static double WAYPOINT_2_Y = 72.0;
    public static double WAYPOINT_2_HEADING_DEG = 90.0;

    // Leg 3: straight line to here.
    public static double PARK_X = 36.0;
    public static double PARK_Y = 36.0;
    public static double PARK_HEADING_DEG = 90.0;

    /** Top motor power for the whole route (0 to 1). Start low, raise it once the route looks right. */
    public static double MAX_POWER = 0.6;

    /** True keeps correcting the final spot with the follower; false lets go once it has braked to a stop. */
    public static boolean HOLD_END = true;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Drawing.init();

        telemetry.addLine("Place the robot on the start pose, then press START.");
        telemetry.addData("start", "(%.1f, %.1f) @ %.0f deg", START_X, START_Y, START_HEADING_DEG);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Build after START so values edited in Panels during INIT are used.
        Pose start = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        Pose waypoint1 = new Pose(WAYPOINT_1_X, WAYPOINT_1_Y, Math.toRadians(WAYPOINT_1_HEADING_DEG));
        Pose curveControl = new Pose(CURVE_CONTROL_X, CURVE_CONTROL_Y);
        Pose waypoint2 = new Pose(WAYPOINT_2_X, WAYPOINT_2_Y, Math.toRadians(WAYPOINT_2_HEADING_DEG));
        Pose park = new Pose(PARK_X, PARK_Y, Math.toRadians(PARK_HEADING_DEG));
        double power = Math.max(0.1, Math.min(1.0, MAX_POWER));

        follower.setStartingPose(start);

        PathChain route = follower.pathBuilder()
                .addPath(new BezierLine(start, waypoint1))
                .setLinearHeadingInterpolation(start.getHeading(), waypoint1.getHeading())
                .addPath(new BezierCurve(Arrays.asList(waypoint1, curveControl, waypoint2)))
                .setLinearHeadingInterpolation(waypoint1.getHeading(), waypoint2.getHeading())
                .addPath(new BezierLine(waypoint2, park))
                .setLinearHeadingInterpolation(waypoint2.getHeading(), park.getHeading())
                .build();

        boolean released = false;
        try {
            follower.followPath(route, power, HOLD_END);

            while (opModeIsActive()) {
                follower.update();

                if (!follower.isBusy() && !HOLD_END && !released) {
                    follower.breakFollowing(); // already stopped: release the motors, once
                    released = true;
                }

                telemetry.addData("busy", follower.isBusy());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
                Drawing.drawDebug(follower);
            }
        } finally {
            follower.setMaxPower(1.0); // the follower settings are shared with the other OpModes
        }
    }
}
