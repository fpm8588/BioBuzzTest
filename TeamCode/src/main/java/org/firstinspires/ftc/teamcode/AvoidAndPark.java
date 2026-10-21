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
import org.firstinspires.ftc.teamcode.util.ObstacleMath;
import org.firstinspires.ftc.teamcode.util.ObstacleTracker;
import org.firstinspires.ftc.teamcode.util.PollenMath;

import java.util.ArrayList;
import java.util.List;

/*
 * AVOIDANDPARK: drives the Leave and Park route, and goes round an opposing robot that the Limelight sees in the way.
 *
 * ROUTE: exactly the Leave and Park route, checks and settings (LeaveAndPark.*: USE_SUPPLIED_ROUTE, ROTATE_180, MAX_POWER,
 * ENFORCE_*, TIMEOUT_SEC ...). With no obstacle it drives it unchanged.
 *
 * SEEING: the Limelight runs a colour pipeline for the OPPOSING alliance's colour (every robot carries alliance-coloured
 * signs, rule R402; the pipeline slot is chosen from ROTATE_180). Each blob's size gives a distance, its angle gives a
 * bearing, and that puts a remembered obstacle on the field map (ObstacleTracker). Sightings on the HIVE and flowers are
 * ignored. See ObstacleSettings and docs/LIMELIGHT_OBSTACLES.md.
 *
 * REACTING (checked every REPLAN_INTERVAL_SEC):
 *   - path clear              -> carry on;
 *   - obstacle in the way     -> ObstacleMath.planDetour builds a dog-leg round it: over to one side before it, alongside
 *                                at a safe distance, back onto the route. The detour only goes where the robot may legally
 *                                stand (walls, and the HIVE and flowers at the ObstacleSettings PLAN_ sizes);
 *   - no way round            -> stop and wait (the corridor beside the wall is narrow, so this is the common answer);
 *                                if the obstacle goes, resume; after BLOCKED_WAIT_SEC give up and stop;
 *   - obstacle suddenly close -> stop at once, then plan as above.
 *
 * LIMITS you must know:
 *   - The camera looks along the robot's heading. On route legs where the robot drives sideways to its heading (legs 1 and 3 of
 *     the supplied route) it cannot see where it is going; it only reacts to what it saw earlier (MEMORY_SEC) or sees ahead.
 *   - Distance from patch size is only good to about +/- 30 percent, so the clearances are generous.
 *   - It only looks for the OPPOSING colour: an alliance partner is not seen, and red or blue NECTAR / tape can fool it
 *     unless the pipeline filters are set well (calibration steps).
 *   - If the obstacle pipeline is not confirmed it REFUSES to move (ObstacleSettings.REQUIRE_VISION, on by default). With it off
 *     it drives the route WITHOUT avoidance and says so loudly.
 *   - Whether the pipeline CAN see the robot in front of it is only known to you: at INIT, put the obstacle in view and check
 *     that a patch is listed as "kept". The screen also shows, during the run, how many blobs the camera found and what became of them.
 *
 * Needs the drive motors, dead wheels and IMU plus the Limelight named "limelight".
 */
@Autonomous(name = "AVOIDANDPARK", group = "Competition")
public class AvoidAndPark extends LinearOpMode {

    private enum Mode { LEG, WAITING, DONE, ABORTED }

    // ---- state ----
    private Follower follower;
    private PathChain[] chains;
    private Pose[][] segments;
    private double power;
    private Mode mode = Mode.LEG;
    private int leg = 0;
    private final List<Pose> via = new ArrayList<>();   // waypoints of the detour being driven, then the goal
    private int viaIndex = 0;
    private boolean onDetour = false;
    private Pose segFrom, segTo;                        // the straight line currently being driven
    private double routeStartedAt, waitSince, lastCheck = -1, startedAt = -1;
    private int detours = 0;
    private String note = "";
    private String lastPlan = "";
    private double lastBlobAt = -1;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        Drawing.init();
        Limelight limelight = new Limelight(hardwareMap);
        boolean visionWanted = ObstacleSettings.ENABLE_AVOIDANCE;
        boolean pipelineOk = visionWanted && limelight.selectPipeline(ObstacleSettings.opponentPipeline(), 3000);
        boolean visionOn = visionWanted && pipelineOk;

        try {
            // Show the checks while waiting, and refresh them so a value changed in Panels shows up straight away.
            while (opModeInInit()) {
                LeaveParkPlan plan = LeaveAndPark.buildPlan();
                boolean supplied = LeaveAndPark.USE_SUPPLIED_ROUTE;
                Pose[][] segs = supplied ? new LeaveAndPark.Paths(follower, LeaveAndPark.ROTATE_180).segments()
                        : LeaveAndPark.generatedSegments(plan);
                telemetry.addLine("AVOIDANDPARK: the Leave and Park route, going round opposing robots the camera sees.");
                LeaveAndPark.showReport(telemetry, LeaveAndPark.evaluate(plan, segs, supplied), supplied);
                telemetry.addData("Limelight", limelight.statusLine());
                telemetry.addData("avoidance", !visionWanted ? "OFF (ENABLE_AVOIDANCE = false)"
                        : pipelineOk ? "ON, pipeline " + ObstacleSettings.opponentPipeline() + " (" + (LeaveAndPark.ROTATE_180 ? "red" : "blue") + " robots)"
                        : "OFF: pipeline " + ObstacleSettings.opponentPipeline() + " NOT confirmed" + (ObstacleSettings.REQUIRE_VISION ? " (the robot will not move)" : ""));
                if (!visionOn) {
                    telemetry.addLine("!! AVOIDANCE IS OFF: the robot will NOT go round anything" + (ObstacleSettings.REQUIRE_VISION ? " (and will not move)." : "."));
                }
                if (visionOn) {
                    limelight.update();
                    Pose first = segs[0][0];
                    List<PollenMath.Blob> blobs = limelight.getBlobs(ObstacleSettings.MAX_STALENESS_MS);
                    ObstacleTracker preview = ObstacleSettings.newTracker(ObstacleSettings.planningGuard());
                    telemetry.addData("frame age (ms)", limelight.getStalenessMs() == Long.MAX_VALUE ? "no result (nothing of this colour in view?)" : String.valueOf(limelight.getStalenessMs()));
                    telemetry.addData("blobs in view", blobs.size());
                    for (PollenMath.Blob b : blobs) {
                        double[] pos = ObstacleSettings.position(first, b);
                        String verdict = pos == null ? "TOO SMALL (below MIN_AREA " + ObstacleSettings.MIN_AREA + ")"
                                : preview.isIgnored(pos[0], pos[1]) ? "IGNORED (lands on the HIVE / a flower)" : "KEPT";
                        telemetry.addData("  patch", "%s: tx %.1f  area %.3f%%  ~%s in",
                                verdict, b.txDeg, b.area * 100,
                                Double.isNaN(ObstacleMath.distanceFromArea(b.area, ObstacleSettings.DISTANCE_K)) ? "?" : String.valueOf(Math.round(ObstacleMath.distanceFromArea(b.area, ObstacleSettings.DISTANCE_K))));
                    }
                    if (blobs.isEmpty()) {
                        telemetry.addLine("  ! NO patch seen. Put the obstacle in front of the camera now: if this stays empty, the pipeline cannot see it"
                                + " (wrong colour for pipeline " + ObstacleSettings.opponentPipeline() + ", not tuned, or nothing built in that slot).");
                    }
                }
                telemetry.update();
                sleep(100);
            }
            if (isStopRequested()) return;

            // Everything again after START so the latest Panels values are used.
            final LeaveParkPlan plan = LeaveAndPark.buildPlan();
            final boolean supplied = LeaveAndPark.USE_SUPPLIED_ROUTE;
            LeaveAndPark.Paths paths = new LeaveAndPark.Paths(follower, LeaveAndPark.ROTATE_180);
            segments = supplied ? paths.segments() : LeaveAndPark.generatedSegments(plan);
            LeaveAndPark.Report report = LeaveAndPark.evaluate(plan, segments, supplied);
            if (ObstacleSettings.ENABLE_AVOIDANCE && ObstacleSettings.REQUIRE_VISION && !visionOn) {
                report.blocking.add("Obstacle pipeline " + ObstacleSettings.opponentPipeline() + " was not confirmed (REQUIRE_VISION is on).");
            }
            if (!report.blocking.isEmpty()) {
                while (opModeIsActive()) { // do nothing, just explain
                    LeaveAndPark.showReport(telemetry, report, supplied);
                    telemetry.update();
                    sleep(200);
                }
                return;
            }

            follower.setStartingPose(segments[0][0]);
            if (supplied) {
                chains = paths.chains();
            } else {
                chains = new PathChain[]{follower.pathBuilder()
                        .addPath(new BezierLine(segments[0][0], segments[0][1]))
                        .setConstantHeadingInterpolation(plan.heading)
                        .build()};
            }
            power = Math.max(0.1, Math.min(1.0, LeaveAndPark.MAX_POWER));

            final LeaveParkPlan planning = ObstacleSettings.planningGuard();
            final ObstacleMath.PoseLegal legal = (x, y, h) -> planning.violation(x, y, h) == null;
            ObstacleTracker tracker = ObstacleSettings.newTracker(planning);
            ElapsedTime run = new ElapsedTime();
            boolean started = false;

            try {
                while (opModeIsActive()) {
                    follower.update();
                    Pose pose = follower.getPose();
                    double now = run.seconds();

                    // ---- see: put what the camera finds on the field map
                    List<ObstacleMath.Obstacle> obstacles = new ArrayList<>();
                    int rawBlobs = 0, bigEnough = 0, kept = 0, ignored = 0;
                    if (visionOn) {
                        limelight.update();
                        for (PollenMath.Blob b : limelight.getBlobs(ObstacleSettings.MAX_STALENESS_MS)) {
                            rawBlobs++;
                            double[] pos = ObstacleSettings.position(pose, b);
                            if (pos == null) continue;
                            bigEnough++;
                            if (tracker.observe(now, pos[0], pos[1])) kept++; else ignored++;
                        }
                        if (rawBlobs > 0) lastBlobAt = now;
                        tracker.expire(now);
                        obstacles = tracker.active(now);
                    }

                    // ---- go
                    if (!started && now >= LeaveAndPark.START_DELAY_SEC) {
                        started = true;
                        startedAt = now;
                        routeStartedAt = now;
                        startLeg();
                    } else if (started && (mode == Mode.LEG) && !follower.isBusy()) {
                        advance();
                    }

                    // ---- react
                    if (started && (mode == Mode.LEG || mode == Mode.WAITING) && visionOn && !obstacles.isEmpty()) {
                        boolean emergency = mode == Mode.LEG && tooClose(pose, obstacles);
                        if (emergency || now - lastCheck >= ObstacleSettings.REPLAN_INTERVAL_SEC) {
                            lastCheck = now;
                            react(now, pose, obstacles, legal, emergency);
                        }
                    } else if (started && mode == Mode.WAITING && visionOn) {
                        // Nothing seen any more: resume.
                        resumeDirect(pose);
                        note = "obstacle gone, resuming";
                    }

                    // ---- safety net
                    if (started && (mode == Mode.LEG)) {
                        String reason = null;
                        String wall = plan.wallViolation(pose.getX(), pose.getY(), pose.getHeading());
                        if (wall != null && report.enforceField) reason = wall;
                        String keep = plan.keepOutViolation(pose.getX(), pose.getY(), pose.getHeading());
                        if (reason == null && keep != null && report.enforceKeepOuts) reason = keep;
                        if (reason == null && LeaveAndPark.offLine(segFrom, segTo, pose.getX(), pose.getY()) > LeaveAndPark.MAX_OFF_LINE_IN) {
                            reason = "robot is more than " + LeaveAndPark.MAX_OFF_LINE_IN + " in off the line it is following";
                        }
                        if (reason != null) abort(reason);
                    }
                    if (started && (mode == Mode.LEG || mode == Mode.WAITING) && now - routeStartedAt > LeaveAndPark.TIMEOUT_SEC) {
                        abort("route took longer than " + LeaveAndPark.TIMEOUT_SEC + " s");
                    }

                    // ---- show
                    telemetry.addData("mode", mode + (onDetour ? " (on a detour)" : "") + (note.isEmpty() ? "" : ": " + note));
                    telemetry.addData("leg", "%d of %d    detours %d", Math.min(leg + 1, chains.length), chains.length, detours);
                    telemetry.addData("pose", "(%.1f, %.1f) %.0f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
                    telemetry.addData("avoidance", visionOn ? obstacles.size() + " obstacle(s)" : "!! OFF: not going round anything");
                    if (visionOn) {
                        // Where detection breaks, if it does: blobs -> big enough -> kept (not on the HIVE/flowers) -> confirmed.
                        telemetry.addData("vision", "frame %s ms | blobs %d, big enough %d, kept %d, ignored %d | tracks %d, confirmed %d | last blob %s",
                                limelight.getStalenessMs() == Long.MAX_VALUE ? "none" : String.valueOf(limelight.getStalenessMs()),
                                rawBlobs, bigEnough, kept, ignored, tracker.trackCount(), obstacles.size(),
                                lastBlobAt < 0 ? "never" : String.format("%.1f s ago", now - lastBlobAt));
                    }
                    for (ObstacleMath.Obstacle o : obstacles) {
                        telemetry.addData("  obstacle", "(%.0f, %.0f)  %.0f in from the robot", o.x, o.y, Math.hypot(o.x - pose.getX(), o.y - pose.getY()));
                    }
                    if (!lastPlan.isEmpty()) telemetry.addData("last plan", lastPlan);
                    telemetry.addData("LEAVE by odometry (gap to wall, in)", "%.1f", plan.wallGap(pose.getX(), pose.getY(), pose.getHeading()));
                    telemetry.addData("PARK by odometry (area in zone, sq in)", "%.0f", plan.overlapWithZone(pose.getX(), pose.getY(), pose.getHeading()));
                    telemetry.update();
                    Drawing.drawDebug(follower);
                }
            } finally {
                follower.breakFollowing();   // motors off
                follower.setMaxPower(1.0);   // the follower settings are shared with the other OpModes
            }
        } finally {
            limelight.stop();
        }
    }

    // ================= route handling =================

    /** Starts driving the route's current leg exactly as the route defines it. */
    private void startLeg() {
        onDetour = false;
        via.clear();
        viaIndex = 0;
        segFrom = segments[leg][0];
        segTo = segments[leg][1];
        mode = Mode.LEG;
        follower.followPath(chains[leg], power, chains.length == 1 && LeaveAndPark.HOLD_END);
    }

    /** A segment finished: on to the next point of a detour, or the next leg, or done. */
    private void advance() {
        if (onDetour && viaIndex < via.size() - 1) {
            Pose from = via.get(viaIndex);
            viaIndex++;
            driveSegment(from, via.get(viaIndex), viaIndex == via.size() - 1 && leg == chains.length - 1);
            return;
        }
        onDetour = false;
        via.clear();
        leg++;
        if (leg >= chains.length) {
            mode = Mode.DONE;
            note = "route finished";
            if (!LeaveAndPark.HOLD_END) follower.breakFollowing();
            return;
        }
        startLeg();
    }

    /** One straight line, heading turning linearly from the first pose's to the second's. */
    private void driveSegment(Pose from, Pose to, boolean lastOfRoute) {
        segFrom = from;
        segTo = to;
        PathChain c = follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
        mode = Mode.LEG;
        follower.followPath(c, power, lastOfRoute && LeaveAndPark.HOLD_END);
    }

    /** From wherever the robot is, straight on to the end of the current leg (after a wait, or when a detour is not needed). */
    private void resumeDirect(Pose pose) {
        Pose goal = segments[leg][1];
        via.clear();
        via.add(new Pose(pose.getX(), pose.getY(), pose.getHeading()));
        via.add(goal);
        viaIndex = 1;
        onDetour = true;
        driveSegment(via.get(0), goal, leg == chains.length - 1);
    }

    private void abort(String reason) {
        mode = Mode.ABORTED;
        note = reason;
        follower.breakFollowing();
    }

    // ================= avoidance =================

    /** Has an obstacle come closer than touching distance plus the emergency margin, and is it ahead of where we are going? */
    private boolean tooClose(Pose pose, List<ObstacleMath.Obstacle> obstacles) {
        double dx = segTo.getX() - pose.getX(), dy = segTo.getY() - pose.getY();
        double len = Math.hypot(dx, dy);
        for (ObstacleMath.Obstacle o : obstacles) {
            double limit = o.radius + ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN + ObstacleSettings.EMERGENCY_MARGIN_IN;
            double d = Math.hypot(o.x - pose.getX(), o.y - pose.getY());
            boolean ahead = len < 1e-6 || ((o.x - pose.getX()) * dx + (o.y - pose.getY()) * dy) > 0;
            if (d < limit && ahead) return true;
        }
        return false;
    }

    /** Is any remaining stretch of what the robot is about to drive blocked by an obstacle? */
    private boolean remainingBlocked(Pose pose, List<ObstacleMath.Obstacle> obstacles) {
        double r = ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN, m = ObstacleSettings.CLEARANCE_MARGIN_IN, t = ObstacleSettings.TRIGGER_RANGE_IN;
        double ax = pose.getX(), ay = pose.getY();
        List<Pose> rest = new ArrayList<>();
        if (onDetour) {
            for (int i = viaIndex; i < via.size(); i++) rest.add(via.get(i));
        } else {
            rest.add(segTo);
        }
        for (Pose p : rest) {
            if (ObstacleMath.isBlocked(ax, ay, p.getX(), p.getY(), obstacles, r, m, t, ObstacleSettings.BLOCK_SLACK_IN)) return true;
            ax = p.getX();
            ay = p.getY();
        }
        return false;
    }

    /** Looks at the obstacles and decides: carry on, go round, or stop and wait. */
    private void react(double now, Pose pose, List<ObstacleMath.Obstacle> obstacles, ObstacleMath.PoseLegal legal, boolean emergency) {
        boolean blocked = emergency || mode == Mode.WAITING || remainingBlocked(pose, obstacles);
        if (!blocked) return;

        if (mode == Mode.LEG) follower.breakFollowing(); // stop while deciding (motors off)

        Pose goal = segments[leg][1];
        ObstacleMath.Plan plan = ObstacleMath.planDetour(pose.getX(), pose.getY(), pose.getHeading(), goal.getX(), goal.getY(), goal.getHeading(),
                obstacles, ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN, ObstacleSettings.CLEARANCE_MARGIN_IN,
                ObstacleSettings.TRIGGER_RANGE_IN, ObstacleSettings.BLOCK_SLACK_IN, legal);
        lastPlan = plan.status + ": " + plan.note + (plan.status == ObstacleMath.Status.DETOUR ? String.format(" (+%.0f in)", plan.extraLengthIn) : "");

        switch (plan.status) {
            case CLEAR:
                resumeDirect(pose);
                note = emergency ? "obstacle close but not in the way, resuming" : "path clear, resuming";
                break;
            case DETOUR:
                if (detours >= ObstacleSettings.MAX_DETOURS) {
                    abort("more than " + ObstacleSettings.MAX_DETOURS + " detours");
                    break;
                }
                detours++;
                via.clear();
                via.add(new Pose(pose.getX(), pose.getY(), pose.getHeading()));
                for (double[] w : plan.waypoints) via.add(new Pose(w[0], w[1], pose.getHeading()));
                via.add(goal);
                viaIndex = 1;
                onDetour = true;
                note = "going round an obstacle";
                driveSegment(via.get(0), via.get(1), false);
                break;
            default: // BLOCKED
                if (mode != Mode.WAITING) {
                    mode = Mode.WAITING;
                    waitSince = now;
                    onDetour = false;
                    follower.breakFollowing();
                }
                note = String.format("blocked, waiting %.1f s", now - waitSince);
                if (now - waitSince > ObstacleSettings.BLOCKED_WAIT_SEC) {
                    abort("blocked for " + ObstacleSettings.BLOCKED_WAIT_SEC + " s with no way round");
                }
                break;
        }
    }
}
