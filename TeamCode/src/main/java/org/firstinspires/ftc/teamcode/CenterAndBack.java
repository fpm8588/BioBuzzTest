package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;

/*
 * Drives from the edge of the ring to the middle, turns 180 degrees in place, and drives back to the edge.
 *
 * Setup: put the robot at the edge of the ring, facing the middle, then press INIT and START.
 * The robot keeps the pose it has at INIT as its start, so no field coordinates need to match anything real.
 *
 * Set DISTANCE_TO_CENTER_IN (inches, edge to middle, measured for the robot's center) before you run it.
 * It can be changed in Panels (Configurables > CenterAndBack) after INIT, or here in the code.
 */
@Configurable
@Autonomous(name = "Center and Back", group = "Competition")
public class CenterAndBack extends LinearOpMode {

    /** Distance from the edge of the ring to the middle, in inches. CHANGE THIS to match your ring. */
    public static double DISTANCE_TO_CENTER_IN = 24.0;

    // Safety limits on the distance above, in inches.
    private static final double MIN_DISTANCE_IN = 1.0;
    private static final double MAX_DISTANCE_IN = 72.0;

    // The robot starts here, facing +x (heading 0), toward the middle. Only the shape of the move matters.
    private static final double START_X = 24.0;
    private static final double START_Y = 72.0;

    private enum State { TO_CENTER, TURN_AROUND, BACK_TO_EDGE, DONE }

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Drawing.init();

        Pose start = new Pose(START_X, START_Y, 0);
        follower.setStartingPose(start);

        telemetry.addLine("Put the robot at the edge of the ring, facing the middle.");
        telemetry.addData("Distance to middle (in)", DISTANCE_TO_CENTER_IN);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Read the distance now, after START, so a value changed in Panels during INIT is used.
        double distance = Math.max(MIN_DISTANCE_IN, Math.min(MAX_DISTANCE_IN, DISTANCE_TO_CENTER_IN));

        Pose middle = new Pose(START_X + distance, START_Y, 0);

        // Edge -> middle, facing the middle the whole way.
        PathChain toCenter = follower.pathBuilder()
                .addPath(new BezierLine(start, middle))
                .setConstantHeadingInterpolation(0)
                .build();

        // Middle -> edge, facing the edge (heading 180 degrees) the whole way.
        PathChain backToEdge = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(middle.getX(), middle.getY(), Math.PI),
                        new Pose(start.getX(), start.getY(), Math.PI)))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        State state = State.TO_CENTER;
        follower.followPath(toCenter, true);

        while (opModeIsActive()) {
            follower.update();

            switch (state) {
                case TO_CENTER:
                    if (!follower.isBusy()) {
                        follower.turn(Math.PI, true); // 180 degrees counter-clockwise (left), in place
                        state = State.TURN_AROUND;
                    }
                    break;
                case TURN_AROUND:
                    if (!follower.isBusy()) {
                        follower.followPath(backToEdge, true);
                        state = State.BACK_TO_EDGE;
                    }
                    break;
                case BACK_TO_EDGE:
                    if (!follower.isBusy()) {
                        state = State.DONE;
                    }
                    break;
                case DONE:
                    break;
            }

            telemetry.addData("state", state);
            telemetry.addData("distance to middle (in)", distance);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
            Drawing.drawDebug(follower);
        }
    }
}
