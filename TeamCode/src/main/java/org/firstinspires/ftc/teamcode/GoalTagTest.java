package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GoalTracker;
import org.firstinspires.ftc.teamcode.util.TagMath;

/*
 * Goal AprilTag check. Does NOT drive: no motor is touched, so it is safe on the bench and on the field.
 *
 * Use it to (1) see that the Limelight runs the AprilTag pipeline, (2) see the tags it finds and their IDs, (3) check that the goal tag
 * is the one chosen, (4) check which way the robot would turn to aim, and (5) check the range against a tape measure, with both methods.
 * Steps are in docs/LIMELIGHT_APRILTAG.md.
 *
 * Needs only the Limelight in the robot configuration, named "limelight". Settings: AprilTagSettings; the camera numbers come from PollenSettings.
 */
@Configurable
@TeleOp(name = "Goal Tag Test", group = "Test")
public class GoalTagTest extends LinearOpMode {

    /** Calibration: put the camera this far from the tag, measured along the floor from the lens to the foot of the tag, inches, to see the error of each range method. 0 = off. */
    public static double KNOWN_DISTANCE_IN = 0.0;

    @Override
    public void runOpMode() {
        GoalTracker goal = GoalTracker.create(hardwareMap);

        telemetry.addData("Limelight", goal.available() ? (goal.pipelineConfirmed() ? "connected, AprilTag pipeline " + AprilTagSettings.APRILTAG_PIPELINE + " selected"
                : "connected but pipeline " + AprilTagSettings.APRILTAG_PIPELINE + " NOT confirmed (is that slot an AprilTag pipeline?)") : "NOT in the robot configuration (name it \"limelight\")");
        telemetry.addData("goal tag ID", AprilTagSettings.goalTagId() < 0 ? "any (set GOAL_TAG_ID_RED / GOAL_TAG_ID_BLUE)"
                : AprilTagSettings.goalTagId() + (LeaveAndPark.ROTATE_180 ? " (blue)" : " (red)"));
        double nearest = TagMath.nearestVisibleDistanceIn(PollenSettings.CAMERA_HEIGHT_IN, PollenSettings.CAMERA_PITCH_DEG,
                PollenSettings.CAMERA_VERTICAL_FOV_DEG, AprilTagSettings.TAG_CENTRE_HEIGHT_IN);
        if (AprilTagSettings.TAG_CENTRE_HEIGHT_IN <= 0) {
            telemetry.addLine("TAG_CENTRE_HEIGHT_IN is not set: the vertical-angle range and the 'closest the tag stays in view' check are off.");
        } else if (Double.isInfinite(nearest)) {
            telemetry.addLine("!! With this camera tilt the tag is NEVER in view: it is higher than the top of the picture at any distance."
                    + " Tilt the camera up (CAMERA_PITCH_DEG) or raise it.");
        } else if (!Double.isNaN(nearest)) {
            telemetry.addData("closest the tag stays in view (in)", "%.0f   (closer than that it leaves the top of the picture)", nearest);
        }
        telemetry.addLine("Press START, then hold the tag in front of the camera.");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                goal.update();
                telemetry.addData("tracker", goal.status());
                telemetry.addData("frame age (ms)", goal.getAgeMs() == Long.MAX_VALUE ? "no result" : String.valueOf(goal.getAgeMs()));
                telemetry.addData("tags in view", goal.getTagsSeen().size());
                for (TagMath.Tag t : goal.getTagsSeen()) {
                    telemetry.addData("  tag " + t.id, "tx %.1f  ty %.1f  area %.3f%%%s", t.txDeg, t.tyDeg, t.area * 100,
                            t.hasPose() ? String.format("  pose range %.0f in", TagMath.poseDistanceIn(t.camXIn, t.camYIn, t.camZIn, PollenSettings.CAMERA_PITCH_DEG)) : "  (no pose)");
                }
                if (goal.hasGoal()) {
                    TagMath.Tag t = null;
                    for (TagMath.Tag c : goal.getTagsSeen()) if (c.id == goal.getId()) { t = c; break; }
                    double byPose = t != null && t.hasPose() ? TagMath.poseDistanceIn(t.camXIn, t.camYIn, t.camZIn, PollenSettings.CAMERA_PITCH_DEG) : Double.NaN;
                    double byAngle = TagMath.tyDistanceIn(goal.getTyDeg(), PollenSettings.CAMERA_HEIGHT_IN, PollenSettings.CAMERA_PITCH_DEG, AprilTagSettings.TAG_CENTRE_HEIGHT_IN);
                    telemetry.addData("range by 3D pose (in)", Double.isNaN(byPose) ? "n/a (no pose: is the tag size set in the pipeline?)" : String.format("%.1f", byPose));
                    telemetry.addData("range by vertical angle (in)", Double.isNaN(byAngle) ? "n/a (set TAG_CENTRE_HEIGHT_IN, check camera height and tilt)" : String.format("%.1f", byAngle));
                    if (t != null && t.hasPose()) {
                        double rise = TagMath.poseHeightAboveCameraIn(t.camYIn, t.camZIn, PollenSettings.CAMERA_PITCH_DEG);
                        telemetry.addData("tag centre height by 3D pose (in)", "%.1f   (use it for TAG_CENTRE_HEIGHT_IN if the pose range is right)", PollenSettings.CAMERA_HEIGHT_IN + rise);
                    }
                    if (KNOWN_DISTANCE_IN > 0) {
                        if (!Double.isNaN(byPose)) telemetry.addData("3D pose error", "%+.1f in (%+.0f%%)", byPose - KNOWN_DISTANCE_IN, 100 * (byPose / KNOWN_DISTANCE_IN - 1));
                        if (!Double.isNaN(byAngle)) telemetry.addData("vertical angle error", "%+.1f in (%+.0f%%)", byAngle - KNOWN_DISTANCE_IN, 100 * (byAngle / KNOWN_DISTANCE_IN - 1));
                    }
                    telemetry.addData("range used (in)", Double.isNaN(goal.getDistanceIn()) ? "unknown" : String.format("%.1f (%s)", goal.getDistanceIn(), goal.getRangeSource()));
                    telemetry.addData("tx (deg, + = right)", "%.1f", goal.getTxDeg());
                    telemetry.addData("aim would", goal.isAimed() ? "NOT turn: AIMED (inside the tolerance)"
                            : goal.getAimTurn() > 0 ? String.format("turn LEFT (power %.2f)", goal.getAimTurn()) : String.format("turn RIGHT (power %.2f)", -goal.getAimTurn()));
                }
                telemetry.update();
            }
        } finally {
            goal.stop();
        }
    }
}
