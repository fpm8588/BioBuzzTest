package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;
import org.firstinspires.ftc.teamcode.util.ObstacleMath;
import org.firstinspires.ftc.teamcode.util.ObstacleTracker;
import org.firstinspires.ftc.teamcode.util.PollenMath;
import org.firstinspires.ftc.teamcode.util.RoutePolyline;

import java.util.ArrayList;
import java.util.List;

/*
 * The avoidance engine shared by AVOIDANDPARK and AVOIDROUTE2: drive a route made of legs (straight lines or Bezier curves),
 * and go round an opposing robot that the Limelight sees in the way. A subclass supplies the route (buildSetup) and its own
 * power / timeout / safety settings; everything else is here.
 *
 * SEEING: the Limelight runs a colour pipeline for the OPPOSING alliance's colour (every robot carries alliance-coloured
 * signs, rule R402; the pipeline slot is chosen from LeaveAndPark.ROTATE_180). Each blob's size gives a distance, its angle gives a
 * bearing, and that puts a remembered obstacle on the field map (ObstacleTracker). Sightings on the HIVE and flowers are
 * ignored. See ObstacleSettings and docs/LIMELIGHT_OBSTACLES.md.
 *
 * REACTING (checked every REPLAN_INTERVAL_SEC, 0.1 s):
 *   - path clear              -> carry on;
 *   - obstacle in the way     -> ObstacleMath.planRoute: a dog-leg round it if there is room (over to one side before it, alongside
 *                                at a safe distance, back onto the route); if the lane beside the obstacle is too narrow (the corridor
 *                                beside the wall is), the LONG WAY round, found by a grid search (ALLOW_LONG_WAY_ROUND, only if it fits
 *                                in the time left). Both only go where the robot may legally stand (walls, and the HIVE and flowers at
 *                                the ObstacleSettings PLAN_ sizes);
 *   - no way at all           -> stop and wait; if the obstacle goes, resume; after BLOCKED_WAIT_SEC give up and stop;
 *   - obstacle suddenly close -> stop at once, then plan as above.
 * Stopping uses the motors' BRAKE, not Pedro's breakFollowing() (which lets them FLOAT and the robot coast into the obstacle).
 *
 * CURVES: a curved leg is followed as the curve until something is in the way. The detour then goes round the obstacle and REJOINS
 * the curve a little past it (the rest of the curve is an exact piece of the original Bezier curve, so the route keeps its shape);
 * a straight leg is rejoined at its end, as before. If the obstacle is so close to the end of a curve that there is no room to
 * rejoin, the detour goes to the end of the leg.
 *
 * NOT SEEING IS NOT GONE: an obstacle is only believed gone if the camera could see where it was and sees nothing there. A robot
 * stopped beside an obstacle cannot see it (blind zone), so it is remembered for HARD_MEMORY_SEC. A flickering pipeline is tolerated:
 * sightings are matched by bearing, not by (noisy) distance, and one seen close counts at once.
 *
 * LIMITS you must know:
 *   - The camera looks along the robot's heading. Wherever the robot drives sideways to its heading (or backwards, or while it turns
 *     through a curve) it cannot see where it is going; it only reacts to what it saw earlier (MEMORY_SEC) or sees ahead.
 *   - Distance from patch size is only good to about +/- 30 percent, so the clearances are generous. The screen shows DISTANCE_K measured
 *     from how the patch grows as the robot approaches, and warns if the set value is far off.
 *   - It only looks for the OPPOSING colour: an alliance partner is not seen, and red or blue NECTAR / tape can fool it
 *     unless the pipeline filters are set well (calibration steps).
 *   - If the obstacle pipeline is not confirmed it REFUSES to move (ObstacleSettings.REQUIRE_VISION, on by default). With it off
 *     it drives the route WITHOUT avoidance and says so loudly.
 *   - Whether the pipeline CAN see the robot in front of it is only known to you: at INIT, put the obstacle in view and check
 *     that a patch is listed as "kept". The screen also shows, during the run, how many blobs the camera found and what became of them.
 *
 * Needs the drive motors, dead wheels and IMU plus the Limelight named "limelight".
 */
abstract class AvoidOpModeBase extends LinearOpMode {

    /** One leg of a route: a straight line or a Bezier curve. */
    static final class Leg {
        Pose start, end;
        /** What Follower.followPath takes. Null when the route was only built for the INIT report. */
        PathChain chain;
        /** The leg as short straight pieces, for distances. */
        RoutePolyline path;
        /** {x, y} of the Bezier control points, ends included; null for a straight line. */
        double[][] control;
        /** How far ahead on the leg the robot is aiming, for "is that obstacle ahead of me?". Infinite = at the end of the leg. */
        double lookAheadIn = Double.POSITIVE_INFINITY;
    }

    /** The route and its checks, rebuilt every INIT loop so a value changed in Panels shows up. */
    static final class Setup {
        LeaveParkPlan plan;
        boolean supplied = true;
        List<Leg> legs;
        LeaveAndPark.Report report;
    }

    // ---- what a subclass provides ----
    abstract String headline();
    /** Build the route and run the checks. Build each leg's PathChain only when withChains is true (after START). */
    abstract Setup buildSetup(Follower follower, boolean withChains);
    abstract double maxPower();
    abstract double startDelaySec();
    abstract boolean holdEnd();
    abstract double timeoutSec();
    abstract double maxOffPathIn();
    /** Show the LEAVE and PARK lines (only meaningful for a route that ends in the loading zone). */
    abstract boolean showParkLines();

    /** Distance between the points handed to the straight-line blocking test along a curve, inches. */
    static final double CURVE_TEST_SPACING_IN = 6.0;
    /** A detour rejoins a curve this far past the last stretch of it that is near an obstacle, inches. */
    static final double REJOIN_BUFFER_IN = 10.0;
    /** A detour never rejoins a curve nearer than this in front of the robot, inches. */
    static final double REJOIN_MIN_AHEAD_IN = 8.0;
    /**
     * While the robot is on a detour a "too close" emergency only fires again once an obstacle is this much closer than it was at the last
     * plan, inches. Without this the robot brakes and replans on every loop for as long as it is inside the emergency distance.
     */
    static final double EMERGENCY_REARM_IN = 5.0;
    /** A new plan whose waypoints are all within this of the detour already being driven is the same detour, inches. */
    static final double SAME_DETOUR_IN = 8.0;

    private enum Mode { LEG, WAITING, DONE, ABORTED }

    // ---- state ----
    private Follower follower;
    private List<Leg> legs;
    private Leg active;                                 // the leg (or the piece of a curve) being followed
    private Leg pendingRejoin;                          // the rest of a curve, driven once the detour reaches it
    private double power;
    private Mode mode = Mode.LEG;
    private int leg = 0;
    private final List<Pose> via = new ArrayList<>();   // waypoints of the detour being driven, then the goal
    private int viaIndex = 0;
    private boolean onDetour = false;
    private RoutePolyline detourPath;                   // the straight line of the detour currently being driven
    private Pose detourTo;
    private double planMinDist = Double.MAX_VALUE;     // distance to the closest obstacle at the last replan
    private double routeStartedAt, waitSince, lastCheck = -1;
    private int detours = 0;
    private String note = "";
    private String lastPlan = "";
    private double lastBlobAt = -1;
    private double curX, curY, curH;                    // the robot's pose this loop, for the camera-view test
    private double kStartArea = -1, kStartX, kStartY, kMeasured = Double.NaN;  // DISTANCE_K measured while approaching

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
                Setup setup = buildSetup(follower, false);
                telemetry.addLine(headline());
                LeaveAndPark.showReport(telemetry, setup.report, setup.supplied);
                telemetry.addData("Limelight", limelight.statusLine());
                telemetry.addData("avoidance", !visionWanted ? "OFF (ENABLE_AVOIDANCE = false)"
                        : pipelineOk ? "ON, pipeline " + ObstacleSettings.opponentPipeline() + " (" + (LeaveAndPark.ROTATE_180 ? "red" : "blue") + " robots)"
                        : "OFF: pipeline " + ObstacleSettings.opponentPipeline() + " NOT confirmed" + (ObstacleSettings.REQUIRE_VISION ? " (the robot will not move)" : ""));
                if (!visionOn) {
                    telemetry.addLine("!! AVOIDANCE IS OFF: the robot will NOT go round anything" + (ObstacleSettings.REQUIRE_VISION ? " (and will not move)." : "."));
                }
                if (visionOn) {
                    limelight.update();
                    Pose first = setup.legs.get(0).start;
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
            Setup setup = buildSetup(follower, true);
            final LeaveParkPlan plan = setup.plan;
            final LeaveAndPark.Report report = setup.report;
            legs = setup.legs;
            if (ObstacleSettings.ENABLE_AVOIDANCE && ObstacleSettings.REQUIRE_VISION && !visionOn) {
                report.blocking.add("Obstacle pipeline " + ObstacleSettings.opponentPipeline() + " was not confirmed (REQUIRE_VISION is on).");
            }
            if (!report.blocking.isEmpty()) {
                while (opModeIsActive()) { // do nothing, just explain
                    LeaveAndPark.showReport(telemetry, report, setup.supplied);
                    telemetry.update();
                    sleep(200);
                }
                return;
            }

            follower.setStartingPose(legs.get(0).start);
            power = Math.max(0.1, Math.min(1.0, maxPower()));

            final LeaveParkPlan planning = ObstacleSettings.planningGuard();
            final ObstacleMath.PoseLegal legal = (x, y, h) -> planning.violation(x, y, h) == null;
            ObstacleTracker tracker = ObstacleSettings.newTracker(planning);
            final double[] visRange = ObstacleSettings.visibleRange();
            // Could the camera see this field point from where the robot is NOW? (Only then does not seeing it mean it is gone.)
            final ObstacleTracker.Visibility visibility = (x, y) -> ObstacleMath.inCameraView(curX, curY, curH, x, y,
                    ObstacleSettings.CAMERA_FORWARD_OFFSET_IN, ObstacleSettings.CAMERA_HORIZONTAL_FOV_DEG, visRange[0], visRange[1]);
            ElapsedTime run = new ElapsedTime();
            boolean started = false;

            try {
                while (opModeIsActive()) {
                    follower.update();
                    Pose pose = follower.getPose();
                    double now = run.seconds();
                    curX = pose.getX();
                    curY = pose.getY();
                    curH = pose.getHeading();

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
                            if (tracker.observe(now, pose.getX(), pose.getY(), pos[0], pos[1])) kept++; else ignored++;
                            watchK(pose, b);
                        }
                        if (rawBlobs > 0) lastBlobAt = now;
                        tracker.expire(now, visibility);
                        obstacles = tracker.active(now);
                    }

                    // ---- go
                    if (!started && now >= startDelaySec()) {
                        started = true;
                        routeStartedAt = now;
                        startLeg();
                    } else if (started && (mode == Mode.LEG) && !follower.isBusy()) {
                        advance();
                    }

                    // ---- react
                    if (started && (mode == Mode.LEG || mode == Mode.WAITING) && visionOn && !obstacles.isEmpty()) {
                        boolean emergency = mode == Mode.LEG && tooClose(pose, obstacles)
                                && (!onDetour || closestObstacle(pose, obstacles) < planMinDist - EMERGENCY_REARM_IN);
                        if (emergency || now - lastCheck >= ObstacleSettings.REPLAN_INTERVAL_SEC) {
                            lastCheck = now;
                            react(now, pose, obstacles, legal, emergency);
                        }
                    } else if (started && mode == Mode.WAITING && visionOn) {
                        // Nothing seen any more: resume.
                        resumeDirect(pose, obstacles);
                        note = "obstacle gone, resuming";
                    }

                    // ---- safety net
                    double offPath = started ? currentPath().distanceTo(pose.getX(), pose.getY()) : 0;
                    if (started && (mode == Mode.LEG)) {
                        String reason = null;
                        String wall = plan.wallViolation(pose.getX(), pose.getY(), pose.getHeading());
                        if (wall != null && report.enforceField) reason = wall;
                        String keep = plan.keepOutViolation(pose.getX(), pose.getY(), pose.getHeading());
                        if (reason == null && keep != null && report.enforceKeepOuts) reason = keep;
                        if (reason == null && offPath > maxOffPathIn()) {
                            reason = "robot is more than " + maxOffPathIn() + " in off the path it is following";
                        }
                        if (reason != null) abort(reason);
                    }
                    if (started && (mode == Mode.LEG || mode == Mode.WAITING) && now - routeStartedAt > timeoutSec()) {
                        abort("route took longer than " + timeoutSec() + " s");
                    }

                    // ---- show
                    telemetry.addData("mode", mode + (onDetour ? " (on a detour)" : "") + (note.isEmpty() ? "" : ": " + note));
                    telemetry.addData("leg", "%d of %d    detours %d", Math.min(leg + 1, legs.size()), legs.size(), detours);
                    telemetry.addData("pose", "(%.1f, %.1f) %.0f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
                    if (started) telemetry.addData("off the path (in)", "%.1f   (the robot stops at %.0f)", offPath, maxOffPathIn());
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
                    if (!Double.isNaN(kMeasured)) {
                        double ratio = kMeasured / ObstacleSettings.DISTANCE_K;
                        telemetry.addData("DISTANCE_K measured while approaching", "%.1f (set %.1f)%s", kMeasured, ObstacleSettings.DISTANCE_K,
                                ratio > 1.4 || ratio < 0.7 ? "   !! the set value looks WRONG: distances will be off by a factor of " + String.format("%.1f", ratio > 1 ? ratio : 1 / ratio) : "");
                    }
                    if (showParkLines()) {
                        telemetry.addData("LEAVE by odometry (gap to wall, in)", "%.1f", plan.wallGap(pose.getX(), pose.getY(), pose.getHeading()));
                        telemetry.addData("PARK by odometry (area in zone, sq in)", "%.0f", plan.overlapWithZone(pose.getX(), pose.getY(), pose.getHeading()));
                    }
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

    private boolean lastLeg() {
        return leg == legs.size() - 1;
    }

    /** The path the robot is meant to be on right now: the detour's straight line, or the leg itself. */
    private RoutePolyline currentPath() {
        return onDetour && detourPath != null ? detourPath : active.path;
    }

    /** Starts driving the route's current leg exactly as the route defines it. */
    private void startLeg() {
        onDetour = false;
        via.clear();
        viaIndex = 0;
        pendingRejoin = null;
        active = legs.get(leg);
        mode = Mode.LEG;
        follower.followPath(active.chain, power, lastLeg() && holdEnd());
    }

    /** A segment finished: on to the next point of a detour, or the rest of a curve, or the next leg, or done. */
    private void advance() {
        if (onDetour && viaIndex < via.size() - 1) {
            Pose from = via.get(viaIndex);
            viaIndex++;
            driveSegment(from, via.get(viaIndex), viaIndex == via.size() - 1 && pendingRejoin == null && lastLeg());
            return;
        }
        onDetour = false;
        via.clear();
        if (pendingRejoin != null) {
            // The detour is back on the curve: carry on along the rest of it.
            active = pendingRejoin;
            pendingRejoin = null;
            mode = Mode.LEG;
            note = "back on the route";
            follower.followPath(chainFor(active), power, lastLeg() && holdEnd());
            return;
        }
        leg++;
        if (leg >= legs.size()) {
            mode = Mode.DONE;
            note = "route finished";
            if (!holdEnd()) follower.breakFollowing();
            return;
        }
        startLeg();
    }

    /** One straight line, heading turning linearly from the first pose's to the second's. */
    private void driveSegment(Pose from, Pose to, boolean lastOfRoute) {
        detourPath = RoutePolyline.line(from.getX(), from.getY(), to.getX(), to.getY());
        detourTo = to;
        PathChain c = follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
        mode = Mode.LEG;
        follower.followPath(c, power, lastOfRoute && holdEnd());
    }

    /** Where a detour should end: a point on the curve past the obstacles, or the end of the leg. Sets pendingRejoin to match. */
    private Pose detourGoal(Pose pose, List<ObstacleMath.Obstacle> obstacles) {
        pendingRejoin = null;
        if (active.control == null) return active.end;
        List<double[]> blockers = new ArrayList<>();
        for (ObstacleMath.Obstacle o : obstacles) {
            blockers.add(new double[]{o.x, o.y, ObstacleMath.clearance(o, ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN, ObstacleSettings.CLEARANCE_MARGIN_IN)});
        }
        double[] r = active.path.rejoinPoint(pose.getX(), pose.getY(), blockers, REJOIN_BUFFER_IN, REJOIN_MIN_AHEAD_IN);
        if (r == null) return active.end;
        // heading there: the leg's linear heading change, by how far along the curve the point is
        double h0 = active.start.getHeading(), h1 = active.end.getHeading();
        double fraction = active.path.fractionAt(r[3]);
        Pose at = new Pose(r[0], r[1], h0 + shortestTurn(h0, h1) * fraction);

        Leg rest = new Leg();
        rest.control = RoutePolyline.splitFrom(active.control, r[2]);
        rest.start = at;
        rest.end = active.end;
        rest.path = RoutePolyline.bezier(rest.control, 60);
        rest.lookAheadIn = active.lookAheadIn;
        pendingRejoin = rest;
        return at;
    }

    /** From wherever the robot is, on to the end of the current leg (or back onto the curve) after a wait, or when a detour is not needed. */
    private void resumeDirect(Pose pose, List<ObstacleMath.Obstacle> obstacles) {
        Pose goal = detourGoal(pose, obstacles);
        via.clear();
        via.add(new Pose(pose.getX(), pose.getY(), pose.getHeading()));
        via.add(goal);
        viaIndex = 1;
        onDetour = true;
        driveSegment(via.get(0), goal, lastLeg() && pendingRejoin == null);
    }

    /** The PathChain of a piece of curve made on the fly after a detour. */
    private PathChain chainFor(Leg l) {
        if (l.chain != null) return l.chain;
        List<Pose> pts = new ArrayList<>();
        for (double[] c : l.control) pts.add(new Pose(c[0], c[1], l.start.getHeading()));
        l.chain = follower.pathBuilder()
                .addPath(new BezierCurve(pts))
                .setLinearHeadingInterpolation(l.start.getHeading(), l.end.getHeading())
                .build();
        return l.chain;
    }

    /** The robot's pose when it has driven s inches along the whole route, heading changing linearly through each leg: {x, y, heading, leg}. */
    static double[] routeAt(List<Leg> legs, double s) {
        double left = Math.max(0, s);
        for (int i = 0; i < legs.size(); i++) {
            Leg l = legs.get(i);
            double len = l.path.length();
            if (left <= len || i == legs.size() - 1) {
                double d = Math.min(left, len);
                double[] p = l.path.pointAt(d);
                double h0 = l.start.getHeading();
                return new double[]{p[0], p[1], h0 + shortestTurn(h0, l.end.getHeading()) * (len == 0 ? 1 : d / len), i};
            }
            left -= len;
        }
        return new double[]{0, 0, 0, 0};
    }

    /**
     * Where along the route an obstacle would NOT be noticed early enough, as one sentence for the INIT screen (null if everywhere is
     * covered). The camera is fixed to the robot and looks along its heading, so on a stretch where the heading is not the direction of
     * travel (a strafe, a curve with its own heading) an obstacle on the path may never be in view until the robot is on top of it.
     * A route point counts as covered if, from some earlier pose at least warnIn route-inches before it, the camera could see it from no
     * more than rangeIn away (an obstacle seen later than that cannot be stopped for: the robot needs about the clearance plus its braking).
     * Points in the first warnIn inches are covered by the view from the start.
     */
    static String coverage(List<Leg> legs, double rangeIn, double warnIn) {
        double total = 0;
        for (Leg l : legs) total += l.path.length();
        double[] vis = ObstacleSettings.visibleRange();
        double step = 2.0;
        int n = (int) Math.floor(total / step) + 1;
        double[][] pose = new double[n][];
        for (int i = 0; i < n; i++) pose[i] = routeAt(legs, i * step);
        boolean[] covered = new boolean[n];
        int coveredCount = 0;
        for (int i = 0; i < n; i++) {
            double need = Math.max(0, i * step - warnIn);
            for (int k = 0; k * step <= need && k < n && !covered[i]; k++) {
                double[] q = pose[k];
                double d = Math.hypot(pose[i][0] - q[0], pose[i][1] - q[1]);
                covered[i] = d <= rangeIn && ObstacleMath.inCameraView(q[0], q[1], q[2], pose[i][0], pose[i][1],
                        ObstacleSettings.CAMERA_FORWARD_OFFSET_IN, ObstacleSettings.CAMERA_HORIZONTAL_FOV_DEG, vis[0], vis[1]);
            }
            if (covered[i]) coveredCount++;
        }
        List<int[]> gaps = new ArrayList<>();   // runs of uncovered samples {from, to}; short ones (under 10 in) are not worth a line
        int from = -1;
        for (int i = 0; i < n; i++) {
            if (!covered[i] && from < 0) from = i;
            if (covered[i] && from >= 0) { if ((i - from) * step >= 10) gaps.add(new int[]{from, i - 1}); from = -1; }
        }
        if (from >= 0 && (n - from) * step >= 10) gaps.add(new int[]{from, n - 1});
        if (gaps.isEmpty()) return null;
        StringBuilder sb = new StringBuilder(String.format("CAMERA COVERAGE: an obstacle is noticed %.0f in ahead (seen from at most %.0f in away) along %d%% of the route."
                + " Not covered", warnIn, rangeIn, Math.round(100.0 * coveredCount / n)));
        for (int g = 0; g < Math.min(3, gaps.size()); g++) {
            double[] a = pose[gaps.get(g)[0]], b = pose[gaps.get(g)[1]];
            sb.append(String.format("%s (%.0f, %.0f) leg %d to (%.0f, %.0f) leg %d", g == 0 ? ":" : ";", a[0], a[1], (int) a[3] + 1, b[0], b[1], (int) b[3] + 1));
        }
        sb.append(": an obstacle there is seen late or not at all.");
        return sb.toString();
    }

    /** The turn from heading a to heading b the way Pedro's linear heading interpolation takes it: the short way round. */
    static double shortestTurn(double a, double b) {
        double d = (b - a) % (2 * Math.PI);
        if (d > Math.PI) d -= 2 * Math.PI;
        if (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }

    private void abort(String reason) {
        mode = Mode.ABORTED;
        note = reason;
        holdStill();
    }

    /**
     * Stop with the motors BRAKING. Pedro's breakFollowing() puts the motors in FLOAT, so the robot coasts several inches, which is
     * exactly the distance the robot does not have when it has just seen an obstacle. Zero teleop drive in brake mode stops it quickly.
     */
    private void holdStill() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    /**
     * Measures DISTANCE_K while the robot drives straight at a patch: the area grows as it gets closer, and the distance it travelled is
     * known from odometry. A set value that is far from the measured one explains an obstacle seen too late or too early.
     */
    private void watchK(Pose pose, PollenMath.Blob b) {
        if (Math.abs(b.txDeg) > 10 || b.area < ObstacleSettings.MIN_AREA) return;
        if (kStartArea < 0 || b.area < kStartArea) {   // (re)start from the smallest patch seen: the farthest point of the approach
            kStartArea = b.area;
            kStartX = pose.getX();
            kStartY = pose.getY();
            return;
        }
        double travelled = (pose.getX() - kStartX) * Math.cos(pose.getHeading()) + (pose.getY() - kStartY) * Math.sin(pose.getHeading());
        if (travelled < 8) return;
        double k = ObstacleMath.kFromApproach(travelled, kStartArea, b.area);
        if (!Double.isNaN(k) && k > 0.2 && k < 100) kMeasured = Double.isNaN(kMeasured) ? k : 0.7 * kMeasured + 0.3 * k;
    }

    // ================= avoidance =================

    /** The point the robot is heading for, for the "is that obstacle ahead of me?" test. */
    private double[] aimPoint(Pose pose) {
        if (onDetour && detourTo != null) return new double[]{detourTo.getX(), detourTo.getY()};
        if (Double.isInfinite(active.lookAheadIn)) return new double[]{active.end.getX(), active.end.getY()};
        return active.path.ahead(pose.getX(), pose.getY(), active.lookAheadIn);
    }

    /** Has an obstacle come closer than touching distance plus the emergency margin, and is it ahead of where we are going? */
    private boolean tooClose(Pose pose, List<ObstacleMath.Obstacle> obstacles) {
        double[] aim = aimPoint(pose);
        double dx = aim[0] - pose.getX(), dy = aim[1] - pose.getY();
        double len = Math.hypot(dx, dy);
        for (ObstacleMath.Obstacle o : obstacles) {
            double limit = o.radius + ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN + ObstacleSettings.EMERGENCY_MARGIN_IN;
            double d = Math.hypot(o.x - pose.getX(), o.y - pose.getY());
            boolean ahead = len < 1e-6 || ((o.x - pose.getX()) * dx + (o.y - pose.getY()) * dy) > 0;
            if (d < limit && ahead) return true;
        }
        return false;
    }

    private static double closestObstacle(Pose pose, List<ObstacleMath.Obstacle> obstacles) {
        double best = Double.MAX_VALUE;
        for (ObstacleMath.Obstacle o : obstacles) best = Math.min(best, Math.hypot(o.x - pose.getX(), o.y - pose.getY()));
        return best;
    }

    /** Is the plan the detour that is already being driven (every waypoint within SAME_DETOUR_IN of it)? */
    private boolean sameDetour(double[][] waypoints) {
        if (!onDetour || via.size() != waypoints.length + 2) return false;
        for (int i = 0; i < waypoints.length; i++) {
            if (Math.hypot(waypoints[i][0] - via.get(i + 1).getX(), waypoints[i][1] - via.get(i + 1).getY()) > SAME_DETOUR_IN) return false;
        }
        return true;
    }

    /** Is any remaining stretch of what the robot is about to drive blocked by an obstacle? */
    private boolean remainingBlocked(Pose pose, List<ObstacleMath.Obstacle> obstacles) {
        double r = ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN, m = ObstacleSettings.CLEARANCE_MARGIN_IN, t = ObstacleSettings.TRIGGER_RANGE_IN;
        double ax = pose.getX(), ay = pose.getY();
        List<double[]> rest = new ArrayList<>();
        if (onDetour) {
            for (int i = viaIndex; i < via.size(); i++) rest.add(new double[]{via.get(i).getX(), via.get(i).getY()});
        } else {
            rest = active.path.remaining(ax, ay, CURVE_TEST_SPACING_IN);
        }
        double travelled = 0;   // how far along the remaining path the piece being tested starts: the trigger range is counted along the path
        for (double[] p : rest) {
            double range = Math.max(0, t - travelled);
            if (ObstacleMath.isBlocked(ax, ay, p[0], p[1], obstacles, r, m, range, ObstacleSettings.BLOCK_SLACK_IN)) return true;
            travelled += Math.hypot(p[0] - ax, p[1] - ay);
            ax = p[0];
            ay = p[1];
        }
        return false;
    }

    /** Looks at the obstacles and decides: carry on, go round, or stop and wait. */
    private void react(double now, Pose pose, List<ObstacleMath.Obstacle> obstacles, ObstacleMath.PoseLegal legal, boolean emergency) {
        boolean blocked = emergency || mode == Mode.WAITING || remainingBlocked(pose, obstacles);
        if (!blocked) return;

        if (mode == Mode.LEG) holdStill(); // stop while deciding, with the brakes on
        planMinDist = closestObstacle(pose, obstacles);

        Pose goal = detourGoal(pose, obstacles);
        double maxExtra = ObstacleSettings.ALLOW_LONG_WAY_ROUND ? ObstacleSettings.LONG_WAY_MAX_EXTRA_IN : 0;
        ObstacleMath.Plan plan = ObstacleMath.planRoute(pose.getX(), pose.getY(), pose.getHeading(), goal.getX(), goal.getY(), goal.getHeading(),
                obstacles, ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN, ObstacleSettings.CLEARANCE_MARGIN_IN,
                ObstacleSettings.TRIGGER_RANGE_IN, ObstacleSettings.BLOCK_SLACK_IN, maxExtra, legal);
        // A long way only makes sense if there is time left to drive it.
        if (plan.status == ObstacleMath.Status.DETOUR && plan.extraLengthIn > 40) {
            double speed = Math.max(10, power * PollenSettings.FULL_POWER_SPEED_IN_S);
            double seconds = (Math.hypot(goal.getX() - pose.getX(), goal.getY() - pose.getY()) + plan.extraLengthIn) / speed;
            if (now + seconds > ObstacleSettings.AUTO_LENGTH_SEC - 1.0) {
                plan = new ObstacleMath.Plan(ObstacleMath.Status.BLOCKED, new double[0][], 0,
                        String.format("the long way needs %.0f s and only %.0f s are left", seconds, ObstacleSettings.AUTO_LENGTH_SEC - now));
            }
        }
        lastPlan = plan.status + ": " + plan.note + (plan.status == ObstacleMath.Status.DETOUR ? String.format(" (+%.0f in)", plan.extraLengthIn) : "");

        switch (plan.status) {
            case CLEAR:
                resumeDirect(pose, obstacles);
                note = emergency ? "obstacle close but not in the way, resuming" : "path clear, resuming";
                break;
            case DETOUR:
                if (!sameDetour(plan.waypoints)) {   // a replan of the detour already being driven is not a new detour
                    if (detours >= ObstacleSettings.MAX_DETOURS) {
                        abort("more than " + ObstacleSettings.MAX_DETOURS + " detours");
                        break;
                    }
                    detours++;
                }
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
                    pendingRejoin = null;
                    holdStill();   // brakes on, and stay put while the obstacle is there
                }
                note = String.format("blocked, waiting %.1f s", now - waitSince);
                if (now - waitSince > ObstacleSettings.BLOCKED_WAIT_SEC) {
                    abort("blocked for " + ObstacleSettings.BLOCKED_WAIT_SEC + " s with no way round");
                }
                break;
        }
    }
}
