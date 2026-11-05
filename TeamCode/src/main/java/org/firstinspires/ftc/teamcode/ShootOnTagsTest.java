package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GoalTracker;
import org.firstinspires.ftc.teamcode.util.ShootPlan;
import org.firstinspires.ftc.teamcode.util.TagMath;

/*
 * Check for "Shoot On Tags". Does NOT drive: no motor is touched, so it is safe on the bench and on the field.
 *
 * It shows the same plan the autonomous will follow (strip position, shooting position, route, every check), and then, with the robot placed BY
 * HAND at the shooting position (measure it with a tape from the HIVE and the walls), it compares what the camera sees with what the plan
 * expects: how far the strip is, how far off-centre it is, and whether the tags are in the camera's view at all.
 *
 * Steps are in docs/SHOOT_ON_TAGS.md. Needs only the Limelight in the robot configuration, named "limelight" (without it, only the plan is shown).
 * The strip follows LeaveAndPark.ROTATE_180: flip it in Panels to see the blue plan.
 */
@Configurable
@TeleOp(name = "Shoot On Tags Test", group = "Test")
public class ShootOnTagsTest extends LinearOpMode {

    /**
     * true = with the robot at the shooting position and the tags in view, also show the tag height the Limelight measures (from the 3D pose),
     * so it can be copied into AprilTagSettings.TAG_CENTRE_HEIGHT_IN.
     */
    public static boolean SHOW_TAG_HEIGHT = true;

    @Override
    public void runOpMode() {
        GoalTracker goal = GoalTracker.create(hardwareMap);

        // The plan is rebuilt while waiting AND while running, so a number changed in Panels shows up straight away.
        while (opModeInInit()) {
            show(goal, false);
            sleep(100);
        }
        if (isStopRequested()) return;
        try {
            while (opModeIsActive()) {
                show(goal, true);
            }
        } finally {
            goal.stop();
        }
    }

    private void show(GoalTracker goal, boolean running) {
        ShootSetup setup = ShootSetup.build();
        goal.update();
        telemetry.addLine(running ? "SHOOT ON TAGS TEST (nothing moves). Put the robot at the shooting position by hand."
                : "SHOOT ON TAGS TEST: the plan. Press START to compare it with the camera.");
        setup.show(telemetry);

        telemetry.addData("camera", goal.status());
        if (!goal.available()) {
            telemetry.update();
            return;
        }
        double h = PollenSettings.CAMERA_HEIGHT_IN, pitch = PollenSettings.CAMERA_PITCH_DEG;
        double tagH = setup.tagHeightIn, d = ShootSettings.SHOOT_DISTANCE_IN;
        boolean inView = TagMath.tagInVerticalView(h, pitch, PollenSettings.CAMERA_VERTICAL_FOV_DEG, tagH, d);
        telemetry.addData("tags in the camera's view at the shooting position?", "%s (tag height %.0f in, %s; camera %.0f in up, tilt %.0f deg %s)",
                inView ? "YES" : "NO", tagH, setup.tagHeightSource, h, Math.abs(pitch), pitch >= 0 ? "down" : "up");
        if (!inView) {
            telemetry.addData("tilt that would centre them", "%.0f deg up (CAMERA_PITCH_DEG = %.0f in PollenSettings)",
                    -TagMath.pitchToCentreTagDeg(h, tagH, d), TagMath.pitchToCentreTagDeg(h, tagH, d));
        }

        telemetry.addData("tags in view", goal.getTagsSeen().size());
        for (TagMath.Tag t : goal.getTagsSeen()) {
            int first = goal.getFirstTagId();
            boolean mine = t.id >= first && t.id < first + TagMath.STRIP_OFFSETS.length;
            telemetry.addData("  tag " + t.id + (mine ? " *" : ""), "tx %.1f  ty %.1f  area %.3f%%", t.txDeg, t.tyDeg, t.area * 100);
        }
        if (goal.hasGoal()) {
            double range = goal.getDistanceIn();
            telemetry.addData("strip tags in view", "%d of %d%s", goal.getTagsUsed(), TagMath.STRIP_OFFSETS.length, goal.isCentred() ? "" : " (NOT centred: set TAG_SIZE_IN)");
            telemetry.addData("camera to strip (in)", Double.isNaN(range) ? "unknown" : String.format("%.1f  (plan %.1f: %+.1f)", range, d, range - d));
            telemetry.addData("strip centre off the crosshair (deg, + = right)", "%.1f  (the plan expects 0: %s)", goal.getTxDeg(),
                    Math.abs(goal.getTxDeg()) <= AprilTagSettings.AIM_TOLERANCE_DEG ? "robot is square on" : "turn the robot or fix STRIP_ALONG_OFFSET_IN");
            if (!Double.isNaN(range) && Math.abs(range - d) > ShootSettings.RANGE_TOLERANCE_IN) {
                telemetry.addLine(String.format("  -> the robot is %.1f in %s the HIVE than planned, or SHOOT_DISTANCE_IN / the HIVE size / the face is off.",
                        Math.abs(range - d), range > d ? "farther from" : "closer to"));
            }
            if (SHOW_TAG_HEIGHT) {
                for (TagMath.Tag t : goal.getTagsSeen()) {
                    if (t.id == goal.getId() && t.hasPose()) {
                        double rise = TagMath.poseHeightAboveCameraIn(t.camYIn, t.camZIn, pitch);
                        telemetry.addData("tag height by the Limelight's 3D pose (in)", "%.1f   (put it in AprilTagSettings.TAG_CENTRE_HEIGHT_IN if the range above is right)", h + rise);
                    }
                }
            }
        } else if (running) {
            telemetry.addLine("No tag of the strip (IDs " + goal.getFirstTagId() + " to " + (goal.getFirstTagId() + TagMath.STRIP_OFFSETS.length - 1)
                    + ") in view. If the plan says they are in view, check the pipeline (slot " + AprilTagSettings.APRILTAG_PIPELINE + "), the tag family and the camera.");
        }
        telemetry.addData("face of the HIVE", ShootPlan.faceName(setup.plan.face));
        telemetry.update();
    }
}
