package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;
import org.firstinspires.ftc.teamcode.util.RoutePolyline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * AVOIDROUTE2: the second route (start, shoot1, a curve up the left side, then across the top), driven with the same
 * obstacle avoidance as AVOIDANDPARK. With nothing in the way it drives the route exactly as exported.
 *
 * ROUTE (field coordinates in inches, headings in degrees; from the Pedro export, same numbers):
 *   start  (56, 8, 90)
 *   leg 1  line   start -> shoot1 (56, 25.3109, 90)                       heading held at 90
 *   leg 2  curve  (56, 25.3109) -> (11.9064, 107.7914), control point (9.9074, 32.1442)   heading 90 -> 180
 *   leg 3  line   (11.9064, 107.7914) -> (56.1014, 116.2739)             heading 180 -> -90 (turns the short way: +90)
 * ROTATE_180 (LeaveAndPark.ROTATE_180, the same switch as every other auto) turns all of it 180 degrees for the blue side.
 *
 * "shoot1" is only a point where the robot stops for an instant: this project has no shooter code, so nothing is fired.
 *
 * AVOIDING: see AvoidOpModeBase. On the curve the robot follows the curve until something is in the way, goes round it, and
 * REJOINS the curve a little past the obstacle (the rest of the curve is an exact piece of the original). On the two straight
 * legs it goes round and on to the end of the leg, as AVOIDANDPARK does.
 *
 * CHECKS (shown at INIT, refused before moving if they fail): the start rules, walls (and the HIVE and flowers if those keep-outs
 * are switched on) along the WHOLE route including the curve, and that the avoidance camera pipeline is confirmed. This route
 * is not meant to end in the loading zone, so it prints no LEAVE / PARK result.
 *
 * Settings below are this route's own; the alliance switch, field checks and keep-outs are shared with Leave and Park, and
 * the camera and avoidance numbers are in ObstacleSettings.
 *
 * Needs the drive motors, dead wheels and IMU plus the Limelight named "limelight".
 */
@Configurable
@Autonomous(name = "AVOIDROUTE2", group = "Competition")
public class AvoidRoute2 extends AvoidOpModeBase {

    /** Drive power, 0 to 1. */
    public static double MAX_POWER = 0.5;

    /** Wait this long after START before moving, seconds. */
    public static double START_DELAY_SEC = 0.0;

    /** true = hold the final pose when the route is done. */
    public static boolean HOLD_END = true;

    /** Give up on the route after this long, seconds. */
    public static double TIMEOUT_SEC = 25.0;

    /**
     * Stop if the robot is more than this far from the path it is following, inches. A curve is followed less tightly than a
     * straight line, so this is looser than Leave and Park's 6. The screen shows the live value ("off the path"): drive the
     * route once with no obstacle and set this a few inches above the largest value you see.
     */
    public static double MAX_OFF_PATH_IN = 8.0;

    /** Warn on the INIT screen if the route comes this close to a flower tower, inches. */
    private static final double FLOWER_WARN_IN = 3.0;

    /** How far ahead on the curve the robot is taken to be heading, for deciding whether an obstacle is in front of it, inches. */
    public static double CURVE_LOOKAHEAD_IN = 12.0;

    /**
     * The route, rewritten for Pedro 2.1.2. The export used Paths.line(a, b).linear(a, b) / .constant(p), Paths.curve(...) and
     * PoseFactory.degrees().of(x, y, h); here those are a BezierLine / BezierCurve with setLinearHeadingInterpolation /
     * setConstantHeadingInterpolation and the pose(...) helper. Same names, same numbers.
     */
    public static class Paths {
        private final Follower follower;
        private final boolean rotate180;

        public final Pose start;
        public final Pose shoot1;
        public final Pose point2Start;
        public final Pose point2;
        public final Pose point2Control1;
        public final Pose point3Start;
        public final Pose point3;

        public Paths(Follower follower, boolean rotate180) {
            this.follower = follower;
            this.rotate180 = rotate180;
            start = pose(56, 8, 90);
            shoot1 = pose(56, 25.3109, 90);
            point2Start = pose(56, 25.3109, 90);
            point2 = pose(11.9064, 107.7914, 180);
            point2Control1 = pose(9.9074, 32.1442, 0); // a control point: its heading is not used
            point3Start = pose(11.9064, 107.7914, 180);
            point3 = pose(56.1014, 116.2739, -90);
        }

        /** Replaces PoseFactory.degrees().of(x, y, headingDegrees); also turns the pose 180 degrees for the other alliance. */
        private Pose pose(double x, double y, double headingDegrees) {
            if (rotate180) {
                return new Pose(LeaveParkPlan.FIELD_IN - x, LeaveParkPlan.FIELD_IN - y, Math.toRadians(headingDegrees + 180.0));
            }
            return new Pose(x, y, Math.toRadians(headingDegrees));
        }

        /** Leg 1: start to shoot1, heading held. */
        public PathChain shoot1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start, shoot1))
                    .setConstantHeadingInterpolation(shoot1.getHeading())
                    .build();
        }

        /** Leg 2: the curve, heading turning linearly from 90 to 180 degrees. */
        public PathChain path2() {
            return follower.pathBuilder()
                    .addPath(new BezierCurve(Arrays.asList(point2Start, point2Control1, point2)))
                    .setLinearHeadingInterpolation(point2Start.getHeading(), point2.getHeading())
                    .build();
        }

        /** Leg 3: across the top, heading turning linearly from 180 to -90 degrees (Pedro takes the short way, +90). */
        public PathChain path3() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(point3Start, point3))
                    .setLinearHeadingInterpolation(point3Start.getHeading(), point3.getHeading())
                    .build();
        }
    }

    @Override
    String headline() {
        return "AVOIDROUTE2: start, shoot1, curve, across the top - going round opposing robots the camera sees.";
    }

    @Override
    Setup buildSetup(Follower follower, boolean withChains) {
        Paths p = new Paths(follower, LeaveAndPark.ROTATE_180);
        Setup s = new Setup();
        s.plan = LeaveAndPark.buildPlan();
        s.supplied = true;

        s.legs = new ArrayList<>();
        s.legs.add(straight(p.start, p.shoot1, withChains ? p.shoot1() : null));
        s.legs.add(curve(p.point2Start, p.point2Control1, p.point2, withChains ? p.path2() : null));
        s.legs.add(straight(p.point3Start, p.point3, withChains ? p.path3() : null));

        // The checks look at the whole route, curve included, as many short straight pieces about 1 in long.
        List<Pose[]> pieces = new ArrayList<>();
        List<Integer> legOf = new ArrayList<>();
        for (int i = 0; i < s.legs.size(); i++) {
            Leg l = s.legs.get(i);
            double len = l.path.length(), h0 = l.start.getHeading(), turn = shortestTurn(h0, l.end.getHeading());
            int n = Math.max(1, (int) Math.ceil(len));
            Pose prev = l.start;
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
        s.report = LeaveAndPark.evaluate(s.plan, pieces.toArray(new Pose[0][]), true, legOfSegment, s.legs.size(), false);
        s.report.label = "AVOIDROUTE2";
        // The flower keep-outs are off by default, so a route that grazes a tower is not refused: say so on the screen instead.
        double[] closest = s.report.closest;
        for (int i = 1; closest != null && i < closest.length; i++) {
            if (closest[i] < FLOWER_WARN_IN) {
                s.report.warnings.add(String.format("The route passes within %.1f in of flower tower %d (its position here is an estimate, +/- 10 in): check on the real field.", closest[i], i));
            }
        }
        // Ahead of the robot by the clearance plus braking, seen from no further than the planner looks (TRIGGER_RANGE_IN).
        double warnIn = ObstacleSettings.OBSTACLE_RADIUS_IN + ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN + ObstacleSettings.CLEARANCE_MARGIN_IN + 5.0;
        String blind = coverage(s.legs, ObstacleSettings.TRIGGER_RANGE_IN, warnIn);
        if (blind != null) s.report.warnings.add(blind);
        Pose[][] ends = new Pose[s.legs.size()][];
        for (int i = 0; i < ends.length; i++) ends[i] = new Pose[]{s.legs.get(i).start, s.legs.get(i).end};
        s.report.segments = ends;   // show the three legs, not the 1 in pieces
        return s;
    }

    private static Leg straight(Pose from, Pose to, PathChain chain) {
        Leg l = new Leg();
        l.start = from;
        l.end = to;
        l.chain = chain;
        l.path = RoutePolyline.line(from.getX(), from.getY(), to.getX(), to.getY());
        return l;
    }

    private static Leg curve(Pose from, Pose control, Pose to, PathChain chain) {
        Leg l = new Leg();
        l.start = from;
        l.end = to;
        l.chain = chain;
        l.control = new double[][]{{from.getX(), from.getY()}, {control.getX(), control.getY()}, {to.getX(), to.getY()}};
        l.path = RoutePolyline.bezier(l.control, 150);
        l.lookAheadIn = CURVE_LOOKAHEAD_IN;
        return l;
    }

    @Override double maxPower() { return MAX_POWER; }
    @Override double startDelaySec() { return START_DELAY_SEC; }
    @Override boolean holdEnd() { return HOLD_END; }
    @Override double timeoutSec() { return TIMEOUT_SEC; }
    @Override double maxOffPathIn() { return MAX_OFF_PATH_IN; }
    @Override boolean showParkLines() { return false; }
}
