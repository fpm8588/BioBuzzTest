package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;
import org.firstinspires.ftc.teamcode.util.RoutePolyline;
import org.firstinspires.ftc.teamcode.util.ShootPlan;
import org.firstinspires.ftc.teamcode.util.TagMath;

import java.util.ArrayList;
import java.util.List;

/**
 * Everything "Shoot On Tags" and "Shoot On Tags Test" need to know before the robot moves: where the strip is, where to shoot from, the
 * route there, and every check on it. Both OpModes build it from the same settings, so the test shows exactly what the autonomous will do.
 * Rebuilt every INIT loop so a value changed in Panels shows up.
 */
final class ShootSetup {

    final ShootPlan plan;
    final LeaveParkPlan guard;
    final Pose start, goal;
    /** start, the waypoints round the HIVE (if any), and the shooting pose. */
    final Pose[] points;
    /** null if there is no route. */
    final double[][] waypoints;
    final List<String> blocking = new ArrayList<>();
    final List<String> warnings = new ArrayList<>();
    final String tagHeightSource;
    final double tagHeightIn;
    private final RoutePolyline[] pieces;

    private ShootSetup(ShootPlan plan, LeaveParkPlan guard, Pose start, Pose goal, double[][] waypoints, String tagHeightSource, double tagHeightIn) {
        this.plan = plan;
        this.guard = guard;
        this.start = start;
        this.goal = goal;
        this.waypoints = waypoints;
        this.tagHeightSource = tagHeightSource;
        this.tagHeightIn = tagHeightIn;
        List<Pose> pts = new ArrayList<>();
        pts.add(start);
        if (waypoints != null) for (double[] w : waypoints) pts.add(new Pose(w[0], w[1], start.getHeading()));
        pts.add(goal);
        this.points = pts.toArray(new Pose[0]);
        this.pieces = new RoutePolyline[points.length - 1];
        for (int i = 0; i < pieces.length; i++) {
            pieces[i] = RoutePolyline.line(points[i].getX(), points[i].getY(), points[i + 1].getX(), points[i + 1].getY());
        }
    }

    static ShootSetup build() {
        ShootPlan plan = ShootPlan.plan(ShootSettings.planConfig());
        LeaveParkPlan guard = ShootSettings.guard();
        Pose start = ShootSettings.startPose();
        Pose goal = new Pose(plan.x, plan.y, plan.heading);

        String[] why = new String[1];
        double[][] wps = ShootPlan.route(start.getX(), start.getY(), start.getHeading(), goal.getX(), goal.getY(), goal.getHeading(),
                (x, y, h) -> guard.violation(x, y, h) == null, why);

        boolean tagHeightSet = AprilTagSettings.TAG_CENTRE_HEIGHT_IN > 0;
        double tagH = tagHeightSet ? AprilTagSettings.TAG_CENTRE_HEIGHT_IN : ShootSettings.TAG_HEIGHT_ESTIMATE_IN;
        ShootSetup s = new ShootSetup(plan, guard, start, goal, wps, tagHeightSet ? "set" : "ESTIMATE", tagH);

        s.blocking.addAll(plan.problems);
        String illegal = guard.violation(goal.getX(), goal.getY(), goal.getHeading());
        if (illegal != null) s.blocking.add("The shooting position is not allowed: " + illegal + ".");
        if (wps == null) s.blocking.add("No way from the start to the shooting position that keeps clear of the HIVE and the walls" + (why[0] == null ? "." : " (" + why[0] + ")."));

        // the start rules are the same as for every autonomous (touching the wall, own half, not in the zone, inside 18 in)
        for (String p : LeaveAndPark.buildPlan().startRuleProblems(start.getX(), start.getY(), start.getHeading())) {
            (LeaveAndPark.ENFORCE_FIELD_CHECKS ? s.blocking : s.warnings).add(p);
        }

        if (ShootSettings.HIVE_CENTRE_X_IN != 72.0 || ShootSettings.HIVE_CENTRE_Y_IN != 72.0) {
            s.warnings.add("The HIVE centre is not the field centre: the shooting position moves with it, but the route's keep-out area stays centred on the field centre.");
        }
        if (ShootSettings.USE_VISION) {
            if (!TagMath.tagInVerticalView(PollenSettings.CAMERA_HEIGHT_IN, PollenSettings.CAMERA_PITCH_DEG, PollenSettings.CAMERA_VERTICAL_FOV_DEG, tagH, ShootSettings.SHOOT_DISTANCE_IN)) {
                s.warnings.add(String.format("The camera cannot see the tags from the shooting position: they are %.0f in up (%s), %.0f in away, and the camera is %.0f in up looking %.0f deg %s. "
                                + "Tilt it to about %.0f deg up to centre them. Until then the robot shoots from the planned position without the camera.",
                        tagH, s.tagHeightSource, ShootSettings.SHOOT_DISTANCE_IN, PollenSettings.CAMERA_HEIGHT_IN, Math.abs(PollenSettings.CAMERA_PITCH_DEG),
                        PollenSettings.CAMERA_PITCH_DEG >= 0 ? "down" : "up",
                        -TagMath.pitchToCentreTagDeg(PollenSettings.CAMERA_HEIGHT_IN, tagH, ShootSettings.SHOOT_DISTANCE_IN)));
            } else if (AprilTagSettings.TAG_SIZE_IN <= 0) {
                s.warnings.add("AprilTagSettings.TAG_SIZE_IN is not set: with only part of the strip in view the camera aims at the middle of the tags it sees, not the middle of the strip.");
            }
        }
        return s;
    }

    /** The route as a Pedro path: straight lines through the waypoints, heading kept until the last line, which turns to the shooting heading. null if start and goal are the same place. */
    PathChain buildChain(Follower follower) {
        if (Math.hypot(goal.getX() - start.getX(), goal.getY() - start.getY()) < 0.5 && points.length == 2) return null;
        PathBuilder b = follower.pathBuilder();
        for (int i = 0; i < points.length - 1; i++) {
            boolean last = i == points.length - 2;
            b.addPath(new BezierLine(points[i], points[i + 1]))
                    .setLinearHeadingInterpolation(points[i].getHeading(), last ? goal.getHeading() : points[i].getHeading());
        }
        return b.build();
    }

    /** How far (x, y) is from the route, inches. */
    double offRoute(double x, double y) {
        double best = Double.MAX_VALUE;
        for (RoutePolyline p : pieces) best = Math.min(best, p.distanceTo(x, y));
        return best;
    }

    double routeLengthIn() {
        double len = 0;
        for (RoutePolyline p : pieces) len += p.length();
        return len;
    }

    /** What the plan is, for the INIT screen of both OpModes. */
    void show(Telemetry t) {
        t.addData("alliance", LeaveAndPark.ROTATE_180 ? "BLUE (the plan turned 180 degrees)" : "RED");
        t.addData("checks", blocking.isEmpty() ? "OK, will run" : "REFUSED (robot will not move)");
        for (String p : blocking) t.addLine("  - " + p);
        for (String w : warnings) t.addLine("  ! " + w);
        t.addData("strip", "centre (%.1f, %.1f) on %s, under the CELLs", plan.stripX, plan.stripY, ShootPlan.faceName(plan.face));
        t.addData("shooting position", "(%.1f, %.1f) facing %.0f deg; camera %.0f in from the strip", plan.x, plan.y, Math.toDegrees(plan.heading), ShootSettings.SHOOT_DISTANCE_IN);
        t.addData("standoff allowed here (in)", "%.1f to %.1f", plan.minStandoffIn, plan.maxStandoffIn);
        t.addData("start", "(%.1f, %.1f) facing %.0f deg", start.getX(), start.getY(), Math.toDegrees(start.getHeading()));
        if (waypoints == null) {
            t.addData("route", "NONE");
        } else {
            StringBuilder sb = new StringBuilder(waypoints.length == 0 ? "straight line" : "round the HIVE via");
            for (double[] w : waypoints) sb.append(String.format(" (%.0f, %.0f)", w[0], w[1]));
            t.addData("route", "%s, %.0f in", sb.toString(), routeLengthIn());
        }
    }
}
