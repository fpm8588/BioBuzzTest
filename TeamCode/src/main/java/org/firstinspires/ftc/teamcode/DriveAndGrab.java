package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;

/*
 * DRIVEANDGRAB: drives the Leave and Park route from its start to its finish, then goes after any pollen it sees.
 *
 *   1. DRIVING:  the same route, checks and settings as Leave and Park (LeaveAndPark.*: USE_SUPPLIED_ROUTE, ROTATE_180,
 *                MAX_POWER, ENFORCE_*, TIMEOUT_SEC, ...). The robot starts at the route's start pose.
 *   2. SETTLING: PAUSE_BEFORE_CHASE_SEC to stand still at the finish.
 *   3. GRABBING: the Pollen Chase behaviour (PollenChaser, settings in PollenSettings): spin to find a pollen (toward the
 *                middle of the field first), turn to
 *                face it, drive up to it, stop at STOP_DISTANCE_IN. Nothing is picked up: this branch has no intake.
 *
 * It all runs on one follower, so the pose at the finish of the route is the pose the chase starts from.
 *
 * Things to know:
 *   - Scoring: LEAVE and PARK are judged at the END of autonomous. Chasing a pollen drives the robot out of the loading
 *     zone, so the points from the route are given up unless the robot is still in the zone when the 30 s end.
 *     This OpMode is a test of the two parts together, not a points routine.
 *   - If the Limelight pipeline is not confirmed, the route still runs and the chase is skipped (the screen says so).
 *   - Every safety check of both parts is active: route checks before and during the drive, then the field guard,
 *     MAX_TRAVEL_IN (from the finish) and RUN_TIMEOUT_SEC (from the start of the chase) during the chase.
 *
 * Needs the drive motors, dead wheels and IMU plus the Limelight named "limelight". Camera setup: docs/LIMELIGHT_POLLEN.md.
 */
@Autonomous(name = "DRIVEANDGRAB", group = "Competition")
public class DriveAndGrab extends LinearOpMode {

    private enum Phase { DRIVING, SETTLING, GRABBING, SKIPPED, ABORTED }

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Drawing.init();
        Limelight limelight = new Limelight(hardwareMap);
        boolean pipelineOk = limelight.selectPipeline(PollenSettings.POLLEN_PIPELINE, 3000);

        try {
            // Show the checks while waiting, and refresh them so a value changed in Panels shows up straight away.
            while (opModeInInit()) {
                LeaveParkPlan plan = LeaveAndPark.buildPlan();
                boolean supplied = LeaveAndPark.USE_SUPPLIED_ROUTE;
                Pose[][] segs = supplied ? new LeaveAndPark.Paths(follower, LeaveAndPark.ROTATE_180).segments()
                        : LeaveAndPark.generatedSegments(plan);
                telemetry.addLine("DRIVEANDGRAB: drive the Leave and Park route, then chase pollen.");
                LeaveAndPark.showReport(telemetry, LeaveAndPark.evaluate(plan, segs, supplied), supplied);

                // The chase part: is the camera ready, and can the robot spin in place where the route ends?
                limelight.update();
                Limelight.Sighting s = limelight.getPollen(PollenSettings.MIN_AREA, PollenSettings.MAX_STALENESS_MS, false, 0, 5.0);
                telemetry.addData("Limelight", limelight.statusLine());
                telemetry.addData("pipeline " + PollenSettings.POLLEN_PIPELINE, pipelineOk ? "selected" : "NOT confirmed: the chase will be skipped");
                telemetry.addData("pollen now", s == null ? "none in view" : String.format("tx %.1f  ty %.1f  dist %.1f in",
                        s.txDeg, s.tyDeg, PollenSettings.distanceIn(s.tyDeg)));
                Pose end = segs[segs.length - 1][1];
                String spin = spinProblem(plan, end);
                if (spin != null) telemetry.addLine("  ! at the finish the chase may stop at once: " + spin);
                telemetry.update();
                sleep(100);
            }
            if (isStopRequested()) return;

            // Everything again after START so the latest Panels values are used.
            final LeaveParkPlan plan = LeaveAndPark.buildPlan();
            final boolean supplied = LeaveAndPark.USE_SUPPLIED_ROUTE;
            LeaveAndPark.Paths paths = new LeaveAndPark.Paths(follower, LeaveAndPark.ROTATE_180);
            Pose[][] segments = supplied ? paths.segments() : LeaveAndPark.generatedSegments(plan);
            LeaveAndPark.Report report = LeaveAndPark.evaluate(plan, segments, supplied);
            if (!report.blocking.isEmpty()) {
                while (opModeIsActive()) { // do nothing, just explain
                    LeaveAndPark.showReport(telemetry, report, supplied);
                    telemetry.update();
                    sleep(200);
                }
                return;
            }

            follower.setStartingPose(segments[0][0]);

            PathChain[] chains;
            if (supplied) {
                chains = paths.chains();
            } else {
                chains = new PathChain[]{follower.pathBuilder()
                        .addPath(new BezierLine(segments[0][0], segments[0][1]))
                        .setConstantHeadingInterpolation(plan.heading)
                        .build()};
            }

            PollenChaser chaser = new PollenChaser(follower, limelight, plan);
            double power = Math.max(0.1, Math.min(1.0, LeaveAndPark.MAX_POWER));
            Phase phase = Phase.DRIVING;
            String note = "";
            int leg = 0;
            boolean started = false;
            double routeStartedAt = 0;
            double settleStartedAt = 0;
            ElapsedTime run = new ElapsedTime();

            try {
                while (opModeIsActive()) {
                    follower.update();
                    Pose pose = follower.getPose();
                    double now = run.seconds();

                    switch (phase) {
                        case DRIVING:
                            if (!started && now >= LeaveAndPark.START_DELAY_SEC) {
                                // Only the last leg holds its end pose; the others just run on into the next.
                                follower.followPath(chains[0], power, chains.length == 1 && LeaveAndPark.HOLD_END);
                                started = true;
                                routeStartedAt = now;
                            } else if (started && !follower.isBusy()) {
                                leg++;
                                if (leg >= chains.length) {
                                    phase = Phase.SETTLING;
                                    settleStartedAt = now;
                                    break;
                                }
                                follower.followPath(chains[leg], power, leg == chains.length - 1 && LeaveAndPark.HOLD_END);
                            }

                            if (started && phase == Phase.DRIVING) {
                                // Live safety net: stop if the robot is where the checks say it must never be, or has wandered.
                                String reason = null;
                                String wall = plan.wallViolation(pose.getX(), pose.getY(), pose.getHeading());
                                if (wall != null && report.enforceField) reason = wall;
                                String keep = plan.keepOutViolation(pose.getX(), pose.getY(), pose.getHeading());
                                if (reason == null && keep != null && report.enforceKeepOuts) reason = keep;
                                if (reason == null && LeaveAndPark.offLine(segments[leg][0], segments[leg][1], pose.getX(), pose.getY())
                                        > LeaveAndPark.MAX_OFF_LINE_IN) {
                                    reason = "robot is more than " + LeaveAndPark.MAX_OFF_LINE_IN + " in off the line it is following (leg " + (leg + 1) + ")";
                                }
                                if (reason == null && now - routeStartedAt > LeaveAndPark.TIMEOUT_SEC) {
                                    reason = "route took longer than " + LeaveAndPark.TIMEOUT_SEC + " s";
                                }
                                if (reason != null) {
                                    phase = Phase.ABORTED;
                                    note = reason;
                                    follower.breakFollowing(); // motors off
                                }
                            }
                            break;

                        case SETTLING:
                            if (now - settleStartedAt >= PollenSettings.PAUSE_BEFORE_CHASE_SEC) {
                                if (pipelineOk) {
                                    // The finish is against a wall, so pollen is most likely toward the middle of the field:
                                    // look that way first instead of spinning the long way round.
                                    Pose here = follower.getPose();
                                    chaser.begin(here, now, towardFieldCentre(here));
                                    phase = Phase.GRABBING;
                                } else {
                                    phase = Phase.SKIPPED;
                                    note = "chase skipped: Limelight pipeline " + PollenSettings.POLLEN_PIPELINE + " was not confirmed";
                                    if (!LeaveAndPark.HOLD_END) follower.breakFollowing();
                                }
                            }
                            break;

                        case GRABBING:
                            chaser.update(now);
                            break;

                        default: // SKIPPED, ABORTED: stand still
                            break;
                    }

                    String line = phase == Phase.GRABBING ? "GRABBING / " + chaser.getState()
                            + (chaser.getNote().isEmpty() ? "" : ": " + chaser.getNote())
                            : phase == Phase.DRIVING ? (started ? "DRIVING leg " + (leg + 1) + " of " + chains.length : "DRIVING (waiting for the start delay)")
                            : phase + (note.isEmpty() ? "" : ": " + note);
                    telemetry.addData("phase", line);
                    if (phase == Phase.GRABBING) {
                        Limelight.Sighting s = chaser.getSighting();
                        telemetry.addData("pollen", s == null ? "none" : String.format("tx %.1f  ty %.1f  area %.3f%%  dist %.1f in",
                                s.txDeg, s.tyDeg, s.area * 100, PollenSettings.distanceIn(s.tyDeg)));
                        telemetry.addData("command", "forward %.2f  turn %.2f", chaser.getForward(), chaser.getTurn());
                    }
                    telemetry.addData("pose", "(%.1f, %.1f) %.0f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
                    telemetry.addData("LEAVE by odometry (gap to wall, in)", "%.1f", plan.wallGap(pose.getX(), pose.getY(), pose.getHeading()));
                    telemetry.addData("PARK by odometry (area in zone, sq in)", "%.0f", plan.overlapWithZone(pose.getX(), pose.getY(), pose.getHeading()));
                    telemetry.addData("Limelight", limelight.statusLine());
                    telemetry.update();
                    Drawing.drawDebug(follower);
                }
            } finally {
                chaser.finish(); // motors off, follower max power back to full (safe even if the chase never began)
            }
        } finally {
            limelight.stop();
        }
    }

    /** +1 if the middle of the field is to the robot's left (search counter-clockwise first), -1 if to its right. */
    private static int towardFieldCentre(Pose at) {
        double centre = LeaveParkPlan.FIELD_IN / 2;
        double bearing = Math.atan2(centre - at.getY(), centre - at.getX()) - at.getHeading();
        bearing = Math.atan2(Math.sin(bearing), Math.cos(bearing)); // wrap to -pi..pi
        return bearing >= 0 ? 1 : -1;
    }

    /** Can the robot turn through every heading at this pose without touching a wall or field element? null = yes. */
    private static String spinProblem(LeaveParkPlan plan, Pose at) {
        if (!PollenSettings.ENABLE_FIELD_GUARD) return null;
        for (int deg = 0; deg < 360; deg += 15) {
            String v = plan.violation(at.getX(), at.getY(), Math.toRadians(deg));
            if (v != null) return v + " if it turns to " + deg + " deg";
        }
        return null;
    }
}
