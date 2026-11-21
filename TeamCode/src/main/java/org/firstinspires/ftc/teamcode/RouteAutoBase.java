package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;
import org.firstinspires.ftc.teamcode.util.RoutePolyline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * The shared part of the "Trace" autonomous routes (Trace1, Trace2, ...): a route that is just a chain of lines and curves from a Pedro
 * Pathing export, driven exactly like Leave and Park. No Limelight, no obstacle avoidance. (A subclass may add one step before the route
 * through the hooks below, as Trace3Shoot does; the plain Traces use none.)
 *
 * A subclass only lists its poses (see Trace1). Everything else is here and is the same as Leave and Park:
 *   - the same settings (Panels > LeaveAndPark): ROTATE_180 (blue = the whole route turned 180 degrees about the field centre), MAX_POWER,
 *     START_DELAY_SEC, HOLD_END, TIMEOUT_SEC, MAX_OFF_LINE_IN, ENFORCE_FIELD_CHECKS, ENFORCE_KEEP_OUTS, ROBOT_LENGTH_IN / ROBOT_WIDTH_IN;
 *   - the same checks, shown on the INIT screen and enforced before the robot moves (the robot refuses to start if one fails) and live while it
 *     drives: the start-position rules, the robot's whole footprint against the field walls along the WHOLE route (curves included), the
 *     HIVE and flower keep-outs when those are switched on, and how far the robot is from the line or curve it is following;
 *   - one extra warning on the INIT screen when the route comes within 3 in of the estimated position of a flower tower.
 *
 * Export -> Pedro 2.1.2: the exports use PoseFactory / Paths.line(a, b).linear(a, b) / .constant(pose) / Paths.curve(...), which do not exist in
 * Pedro 2.1.2. Here a leg is a BezierLine or BezierCurve with setLinearHeadingInterpolation (heading turns from the start pose's to the end
 * pose's) or setConstantHeadingInterpolation (.constant(pose) = hold that pose's heading). The numbers are the export's.
 */
abstract class RouteAutoBase extends LinearOpMode {

    /** Settings that only the Trace routes use (Panels > RouteAutoBase). */
    @Configurable
    public static class Settings {
        /**
         * Stop if the robot is more than this far from a CURVE it is following, inches. A curve is followed less tightly than a line, so this is
         * looser than LeaveAndPark.MAX_OFF_LINE_IN (used for the lines). The screen shows the live value ("off the path").
         */
        public static double MAX_OFF_CURVE_IN = 8.0;
    }

    /** One leg of the route: a straight line, or a curve with one control point. */
    protected static final class Leg {
        final Pose from, to;
        final Pose control;               // null for a line
        final boolean constantHeading;    // true: hold the END pose's heading all the way; false: turn linearly from the start's heading to the end's
        final RoutePolyline path;

        Leg(Pose from, Pose control, Pose to, boolean constantHeading) {
            this.from = from;
            this.control = control;
            this.to = to;
            this.constantHeading = constantHeading;
            this.path = control == null
                    ? RoutePolyline.line(from.getX(), from.getY(), to.getX(), to.getY())
                    : RoutePolyline.bezier(new double[][]{{from.getX(), from.getY()}, {control.getX(), control.getY()}, {to.getX(), to.getY()}}, 150);
        }

        double heading0() {
            return constantHeading ? to.getHeading() : from.getHeading();
        }

        double headingTurn() {
            return constantHeading ? 0 : shortestTurn(from.getHeading(), to.getHeading());
        }

        double maxOffIn() {
            return control == null ? LeaveAndPark.MAX_OFF_LINE_IN : Math.max(LeaveAndPark.MAX_OFF_LINE_IN, Settings.MAX_OFF_CURVE_IN);
        }
    }

    /** The turn from heading a to heading b the way Pedro's linear heading interpolation takes it: the short way round. */
    static double shortestTurn(double a, double b) {
        double d = (b - a) % (2 * Math.PI);
        if (d > Math.PI) d -= 2 * Math.PI;
        if (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }

    // ---- what a subclass provides ----

    /** The name shown on the INIT screen. */
    protected abstract String title();

    /** The legs, in order, built with pose(), line() and curve(). Called every INIT loop and again after START, so Panels values apply. */
    protected abstract List<Leg> buildLegs();

    /** true if the route is meant to end in the loading zone: only then the LEAVE / PARK lines and warnings are shown. */
    protected boolean endsInZone() {
        return true;
    }

    // ---- optional hooks (all do nothing here, so the plain Trace routes are unchanged) ----

    /** Called once at INIT, before the checks are shown: create any extra hardware here (a missing device then fails at INIT, not at START). */
    protected void onInit() {
    }

    /** Called every INIT loop: add any extra lines to the INIT screen. */
    protected void initTelemetry() {
    }

    /**
     * Called once after START and after the checks pass, with the robot standing still, BEFORE the route starts. Do the extra step here and
     * return when it is done; it must also return promptly when opModeIsActive() turns false. The route's timers start after it returns.
     */
    protected void beforeRoute() {
    }

    /** Called once when the OpMode ends for any reason (route done, aborted, STOP pressed, an error): switch any extra hardware off. */
    protected void onFinish() {
    }

    // ---- helpers for the subclasses ----

    /** x, y in inches and the heading in DEGREES (what the export gives); turned 180 degrees about the field centre when LeaveAndPark.ROTATE_180 is set. */
    protected final Pose pose(double x, double y, double headingDegrees) {
        if (LeaveAndPark.ROTATE_180) {
            return new Pose(LeaveParkPlan.FIELD_IN - x, LeaveParkPlan.FIELD_IN - y, Math.toRadians(headingDegrees + 180.0));
        }
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }

    /** A straight line. constantHeading = the export's .constant(endPose); false = its .linear(startPose, endPose). */
    protected final Leg line(Pose from, Pose to, boolean constantHeading) {
        return new Leg(from, null, to, constantHeading);
    }

    /** A curve with one control point (a quadratic Bezier, the export's Paths.curve(start, control, end)). */
    protected final Leg curve(Pose from, Pose control, Pose to, boolean constantHeading) {
        return new Leg(from, control, to, constantHeading);
    }

    // ---- building the plan and its checks ----

    private static final double FLOWER_WARN_IN = 3.0;

    private static final class Setup {
        LeaveParkPlan plan;
        List<Leg> legs;
        LeaveAndPark.Report report;
    }

    private Setup build() {
        Setup s = new Setup();
        s.plan = LeaveAndPark.buildPlan();
        s.legs = buildLegs();

        // The checks look at the whole route, curves included, as many short straight pieces about 1 in long.
        List<Pose[]> pieces = new ArrayList<>();
        List<Integer> legOf = new ArrayList<>();
        for (int i = 0; i < s.legs.size(); i++) {
            Leg l = s.legs.get(i);
            double len = l.path.length(), h0 = l.heading0(), turn = l.headingTurn();
            int n = Math.max(1, (int) Math.ceil(len));
            Pose prev = l.from;
            prev = new Pose(prev.getX(), prev.getY(), h0);
            for (int k = 1; k <= n; k++) {
                double d = len * k / n;
                double[] pt = l.path.pointAt(d);
                Pose next = new Pose(pt[0], pt[1], h0 + turn * (d / len));
                pieces.add(new Pose[]{prev, next});
                legOf.add(i);
                prev = next;
            }
        }
        int[] legOfSegment = new int[legOf.size()];
        for (int i = 0; i < legOfSegment.length; i++) legOfSegment[i] = legOf.get(i);
        s.report = LeaveAndPark.evaluate(s.plan, pieces.toArray(new Pose[0][]), true, legOfSegment, s.legs.size(), endsInZone());
        s.report.label = title();

        // Flower keep-outs are off by default, so a route that grazes a tower is not refused: say so on the screen instead.
        double[] closest = s.report.closest;
        for (int i = 1; closest != null && i < closest.length; i++) {
            if (closest[i] < FLOWER_WARN_IN) {
                s.report.warnings.add(String.format("The route passes within %.1f in of flower tower %d (its position here is an estimate, +/- 10 in): check on the real field.", closest[i], i));
            }
        }
        Pose[][] ends = new Pose[s.legs.size()][];
        for (int i = 0; i < ends.length; i++) ends[i] = new Pose[]{s.legs.get(i).from, s.legs.get(i).to};
        s.report.segments = ends;   // show the legs, not the 1 in pieces
        return s;
    }

    private PathChain chainFor(Follower follower, Leg l) {
        PathBuilder b = follower.pathBuilder();
        if (l.control == null) {
            b.addPath(new BezierLine(l.from, l.to));
        } else {
            b.addPath(new BezierCurve(Arrays.asList(l.from, l.control, l.to)));
        }
        if (l.constantHeading) {
            b.setConstantHeadingInterpolation(l.to.getHeading());
        } else {
            b.setLinearHeadingInterpolation(l.from.getHeading(), l.to.getHeading());
        }
        return b.build();
    }

    // ---- running it ----

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Drawing.init();
        onInit();

        // Show the checks while waiting, and refresh them so a value changed in Panels shows up straight away.
        while (opModeInInit()) {
            Setup s = build();
            telemetry.addLine(title() + ": drive the route and stop. No camera, no avoidance.");
            LeaveAndPark.showReport(telemetry, s.report, true);
            initTelemetry();
            telemetry.update();
            sleep(100);
        }
        if (isStopRequested()) return;

        // Everything again after START so the latest Panels values are used.
        final Setup s = build();
        final LeaveParkPlan plan = s.plan;
        final LeaveAndPark.Report report = s.report;
        if (!report.blocking.isEmpty()) {
            while (opModeIsActive()) { // do nothing, just explain
                LeaveAndPark.showReport(telemetry, report, true);
                telemetry.update();
                sleep(200);
            }
            return;
        }

        follower.setStartingPose(s.legs.get(0).from);
        PathChain[] chains = new PathChain[s.legs.size()];
        for (int i = 0; i < chains.length; i++) chains[i] = chainFor(follower, s.legs.get(i));

        double power = Math.max(0.1, Math.min(1.0, LeaveAndPark.MAX_POWER));
        int leg = 0;
        boolean started = false;
        boolean finished = false;
        boolean released = false;
        String aborted = null;
        ElapsedTime clock = new ElapsedTime();

        try {
            beforeRoute();
            clock.reset(); // the start delay and the route timeout count from here, after any step before the route
            while (opModeIsActive()) {
                follower.update();
                Pose pose = follower.getPose();

                if (!started && clock.seconds() >= LeaveAndPark.START_DELAY_SEC) {
                    // Only the last leg holds its end pose; the others just run on into the next.
                    follower.followPath(chains[0], power, chains.length == 1 && LeaveAndPark.HOLD_END);
                    started = true;
                    clock.reset();
                } else if (started && !finished && aborted == null && !follower.isBusy()) {
                    leg++;
                    if (leg >= chains.length) {
                        finished = true;
                    } else {
                        follower.followPath(chains[leg], power, leg == chains.length - 1 && LeaveAndPark.HOLD_END);
                    }
                }

                double offPath = started && !finished ? s.legs.get(leg).path.distanceTo(pose.getX(), pose.getY()) : 0;
                if (started && !finished && aborted == null) {
                    // Live safety net: stop if the robot is where the checks say it must never be, or has wandered.
                    String reason = null;
                    String wall = plan.wallViolation(pose.getX(), pose.getY(), pose.getHeading());
                    if (wall != null && report.enforceField) reason = wall;
                    String keep = plan.keepOutViolation(pose.getX(), pose.getY(), pose.getHeading());
                    if (reason == null && keep != null && report.enforceKeepOuts) reason = keep;
                    if (reason == null && offPath > s.legs.get(leg).maxOffIn()) {
                        reason = String.format("robot is more than %.0f in off the %s it is following (leg %d)", s.legs.get(leg).maxOffIn(),
                                s.legs.get(leg).control == null ? "line" : "curve", leg + 1);
                    }
                    if (reason == null && clock.seconds() > LeaveAndPark.TIMEOUT_SEC) {
                        reason = "route took longer than " + LeaveAndPark.TIMEOUT_SEC + " s";
                    }
                    if (reason != null) {
                        aborted = reason;
                        follower.breakFollowing(); // motors off
                    }
                }

                if (finished && aborted == null && !LeaveAndPark.HOLD_END && !released) {
                    follower.breakFollowing(); // already stopped: release the motors, once
                    released = true;
                }

                telemetry.addData("state", aborted != null ? "ABORTED: " + aborted
                        : !started ? "waiting for start delay"
                        : finished ? "done" : "driving leg " + (leg + 1) + " of " + chains.length);
                telemetry.addData("x", "%.1f", pose.getX());
                telemetry.addData("y", "%.1f", pose.getY());
                telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
                if (started && !finished) telemetry.addData("off the path (in)", "%.1f", offPath);
                if (endsInZone()) {
                    // What the wheels say, not what the referee sees: check the real robot against the tape.
                    telemetry.addData("LEAVE by odometry (gap to wall, in)", "%.1f", plan.wallGap(pose.getX(), pose.getY(), pose.getHeading()));
                    telemetry.addData("PARK by odometry (area in zone, sq in)", "%.0f", plan.overlapWithZone(pose.getX(), pose.getY(), pose.getHeading()));
                }
                telemetry.update();
                Drawing.drawDebug(follower);
            }
        } finally {
            try {
                onFinish();
            } finally {
                follower.setMaxPower(1.0); // the follower settings are shared with the other OpModes
            }
        }
    }
}
