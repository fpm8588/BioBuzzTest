package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
 * The turn is done as two 90 degree steps with a lower position-hold gain and a power cap so it stays steady
 * (see TURN_KP and TURN_MAX_POWER).
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

    /**
     * Predictive-braking kP used ONLY while turning in place. The normal kP is much higher and, while spinning, makes
     * the robot fight tiny apparent position drift sideways. That position correction takes priority over the
     * turn in Pedro's wheel-power mixing, so the robot shakes instead of turning. Lower = gentler hold.
     */
    public static double TURN_KP = 0.2;

    /**
     * After the last move, false = stop and let go (no shaking; the robot has already braked to a stop), true =
     * keep actively holding the final spot using HOLD_KP.
     */
    public static boolean HOLD_FINAL_POSITION = false;

    /** Position-hold gain used while holding the final spot (only when HOLD_FINAL_POSITION is true). */
    public static double HOLD_KP = 0.2;

    /**
     * Motor power cap (0 to 1) while turning in place. A full-power 180 degree turn overshoots and oscillates with
     * a heading PIDF that was tuned on small turns, so the turn is slowed down. Lower = slower and steadier.
     */
    public static double TURN_MAX_POWER = 0.5;

    // Safety limits on the distance above, in inches.
    private static final double MIN_DISTANCE_IN = 1.0;
    private static final double MAX_DISTANCE_IN = 72.0;

    // The robot starts here, facing +x (heading 0), toward the middle. Only the shape of the move matters.
    private static final double START_X = 24.0;
    private static final double START_Y = 72.0;

    private enum State { TO_CENTER, TURN_TO_90, TURN_TO_180, BACK_TO_EDGE, DONE }

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
        double turnPower = Math.max(0.15, Math.min(1.0, TURN_MAX_POWER));

        // The follower shares this object with Constants, so it must be restored when the OpMode ends.
        PredictiveBrakingCoefficients braking = follower.getConstants().predictiveBrakingCoefficients;
        double driveKp = braking.P;
        double turnKp = Math.max(0.02, Math.min(driveKp, TURN_KP));

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

        try {
            State state = State.TO_CENTER;
            follower.followPath(toCenter, true);

            while (opModeIsActive()) {
                follower.update();

                switch (state) {
                    case TO_CENTER:
                        if (!follower.isBusy()) {
                            follower.setMaxPower(turnPower);
                            braking.P = turnKp;
                            // Turn left in place, first to 90 degrees. Two steps avoid a single exact-180 turn, whose
                            // direction is ambiguous (the heading error flips sign with a tiny bit of noise).
                            follower.turnTo(Math.PI / 2);
                            state = State.TURN_TO_90;
                        }
                        break;
                    case TURN_TO_90:
                        if (!follower.isBusy()) {
                            follower.turnTo(Math.PI);
                            state = State.TURN_TO_180;
                        }
                        break;
                    case TURN_TO_180:
                        if (!follower.isBusy()) {
                            follower.setMaxPower(1.0); // full speed and the normal gain again for the drive back
                            braking.P = driveKp;
                            follower.followPath(backToEdge, true);
                            state = State.BACK_TO_EDGE;
                        }
                        break;
                    case BACK_TO_EDGE:
                        if (!follower.isBusy()) {
                            if (HOLD_FINAL_POSITION) {
                                // Keep correcting the final spot, but with a gentle gain so it doesn't shake.
                                braking.P = Math.max(0.02, Math.min(driveKp, HOLD_KP));
                            } else {
                                // Already stopped: release the motors so nothing keeps correcting (and shaking).
                                follower.breakFollowing();
                            }
                            state = State.DONE;
                        }
                        break;
                    case DONE:
                        break;
                }

                telemetry.addData("state", state);
                telemetry.addData("kP now", braking.P);
                telemetry.addData("distance to middle (in)", distance);
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
                Drawing.drawDebug(follower);
            }
        } finally {
            braking.P = driveKp; // restore the normal gain, even if STOP is pressed mid-turn
            follower.setMaxPower(1.0);
        }
    }
}
