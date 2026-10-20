package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;

import java.util.ArrayList;
import java.util.List;

/*
 * BIOBUZZ autonomous: earns LEAVE (3) and AUTO PARK (5) and nothing else.
 *
 *   LEAVE = the robot is no longer touching the perimeter wall at the end of autonomous.
 *   PARK  = the robot is at least partially in its own LOADING ZONE at the end of autonomous.
 *
 * ROUTE: by default (USE_SUPPLIED_ROUTE = true) it drives the team's three-path route in the Paths class below:
 *   path1: (56, 8, 90 deg) -> (23.39, 23.97, 90 deg)       diagonal off the bottom wall
 *   path2: (23.39, 23.97) -> (23.46, 106.04), 90 deg        straight up the field beside the red wall
 *   path3: (23.46, 106.04, 90 deg) -> (11.42, 119.97, 0 deg) into the loading zone, turning to face away from the wall
 * That route came from a Pedro Pathing export that uses a newer API (com.pedropathing.api.Paths, PoseFactory) than the
 * Pedro 2.1.2 this project builds with, so Paths is the same poses and lines rewritten with the 2.1.2 calls
 * (BezierLine + linear heading interpolation). The numbers are unchanged. ROTATE_180 turns the whole route 180 degrees
 * about the field centre for the other alliance.
 *
 * The original route (USE_SUPPLIED_ROUTE = false) starts with its back edge against the wall, just beside its LOADING
 * ZONE, and drives one short straight line (about 23 inches, no turning) that backs it off the wall and into the zone.
 * Its start and end poses are worked out from the zone rectangle by util/LeaveParkPlan.
 *
 * Field elements and how they are avoided:
 *   - Perimeter walls: the plan keeps the robot's whole footprint inside the field, and the route ends clear of
 *     the wall by LEAVE_CLEARANCE_IN.
 *   - HIVE structure in the middle of the field and the four FLOWER towers (they stand near the perimeter walls in a
 *     pinwheel pattern, NOT in the middle): a keep-out square around each is checked along the whole planned line
 *     before the robot moves, and against the live position every loop while it moves. The original route stays at the
 *     field edge, over 35 in from the HIVE keep-out and about 20 in from the closest flower. The supplied route
 *     passes about 10 in from the HIVE and, by the estimated positions, 1.9 in from flower 2: see the INIT screen.
 *   - Other robots can not be planned for. Agree with your alliance partner which of you leaves first, and use
 *     START_DELAY_SEC if you need to wait.
 * If a check marked as enforced fails, the robot does not move and the screen says why. If the robot ever leaves the
 * route, it stops. For the supplied route the field-wall and start-rule checks are enforced (ENFORCE_FIELD_CHECKS) but
 * the HIVE and FLOWER keep-outs only warn (ENFORCE_KEEP_OUTS = false), because those positions are estimates read off a
 * drawing and the route was drawn on the real field. The INIT screen lists how close the route gets to each element;
 * measure before you trust it, and turn ENFORCE_KEEP_OUTS on to make the robot refuse and stop instead.
 *
 * Robot set-up rules (manual, "ROBOTS must be set up correctly on the FIELD") and how this OpMode handles each:
 *   A. fully on its own alliance side (columns A, B, C = red, D, E, F = blue): checked by the plan. The side comes
 *      from where the loading zone is, and the plan refuses a start that is not fully inside that half.
 *   B. not attached to or hanging from a field element: you check this by eye.
 *   C. touching the perimeter wall: the start pose puts the back edge exactly on the wall.
 *   D. not touching or inside a FLOWER's scoring volume: each flower has a keep-out square (FLOWER_HALF_IN plus
 *      FLOWER_MARGIN_IN around its centre) that the start, the route and the end must stay out of. Check by eye too.
 *      The flower keep-outs are currently switched OFF (both set to 0), so only the walls (and the HIVE) are checked.
 *   E. not in the LOADING ZONE: the start is beside the zone with START_GAP_IN (at least 1 in) of space.
 *   F. inside the starting configuration (18 in cube): the plan refuses a robot size over 18 in. Height and any
 *      parts that unfold are for you to check.
 *   G. in contact with exactly 4 POLLEN pre-loads, on your own side: you do this. Do not put a pre-load between the
 *      robot and the zone or in the robot's path (the robot drives away from the wall, toward the zone side).
 *   H. fully motionless after INIT: the only motor command before START is zero power (from building the follower).
 *
 * VERIFY BEFORE USE (see the numbers marked ASSUMED): the game manual could not be read from the build machine, so
 * the loading zone position, the flower positions and the HIVE size came from search results and a field drawing,
 * not the manual. The positions are in a frame where x runs from the red alliance wall (x = 0) to the blue wall
 * (x = 144), and y runs along the walls.
 * 1. Measure the LOADING ZONE tape on your field and fix ZONE_* (inches in that frame). Per the manual's columns,
 *    the red zone is in column A and the blue zone in column F.
 * 2. Measure the centre of each FLOWER tower and fix FLOWER_n_X / FLOWER_n_Y. The defaults were read off a drawing
 *    and can be about 10 in out.
 * 3. Set ROBOT_LENGTH_IN / ROBOT_WIDTH_IN to the real chassis size in the start configuration (at most 18 x 18).
 *
 * Setup: the Driver Station prints the exact start pose at INIT. For the supplied route that is (56, 8) facing 90 deg
 * (toward +y), back edge flat on the bottom wall. For the original route it is beside the zone, back edge flat on the
 * wall, facing away from it.
 * All numbers can be changed in Panels (Configurables > LeaveAndPark) during INIT.
 */
@Configurable
@Autonomous(name = "Leave and Park", group = "Competition")
public class LeaveAndPark extends LinearOpMode {

    // ASSUMED: the red LOADING ZONE is on tile A5 and the blue one on tile F2 (a 180 degree turn of each other),
    // about 23 in along the wall and 11 in deep. In this frame the red wall is x = 0, x grows toward the blue wall, and
    // y grows counter-clockwise (right-handed, same as Pedro). Tile row 5 is y 96 to 120. The supplied route ends at
    // (11.4, 120) beside this wall, which agrees with the zone being here.
    // Measure your field and correct these. Rectangle corners in inches, before ROTATE_180.
    public static double ZONE_X_MIN = 0.0;
    public static double ZONE_Y_MIN = 96.5;
    public static double ZONE_X_MAX = 11.0;
    public static double ZONE_Y_MAX = 119.5;

    /**
     * false = use the zone rectangle as typed (the red zone, columns A-C, the x 0 to 72 half);
     * true = turn it 180 degrees about the field centre (the blue zone, columns D-F, the x 72 to 144 half).
     */
    public static boolean ROTATE_180 = false;

    /**
     * Original route only. Which side of the zone to start on along the wall: -1 or +1. Default -1 (toward smaller y on
     * the red wall), which is away from the flower at about (40, 128). The plan refuses a side that hits a flower or
     * leaves your half.
     */
    public static int START_SIDE = -1;

    /**
     * Real robot size in the start configuration, inches (18 x 18 is the largest allowed to start). 16 x 16 matches the
     * supplied route's start at y = 8 with the robot facing 90 deg (half of 16 is 8, so the back edge is on the wall).
     * Set these to your real chassis.
     */
    public static double ROBOT_LENGTH_IN = 16.0; // front to back
    public static double ROBOT_WIDTH_IN = 16.0;  // side to side

    /** Gap between the robot and the zone edge at the start, inches. At least 1: the robot must not be in the zone. */
    public static double START_GAP_IN = 2.0;

    /** How far the robot's back edge ends up from the wall. Must be above 0 for LEAVE; zone depth minus this is the overlap. */
    public static double LEAVE_CLEARANCE_IN = 5.0;

    // ASSUMED: the four FLOWER tower centres, read off a field drawing (+/- 10 in), in the same frame as the zone.
    // They stand near the walls, and the set looks the same after a 180 degree turn, so ROTATE_180 does not move them.
    public static double FLOWER_1_X = 40.0,  FLOWER_1_Y = 128.0;
    public static double FLOWER_2_X = 10.0,  FLOWER_2_Y = 44.0;
    public static double FLOWER_3_X = 104.0, FLOWER_3_Y = 16.0;
    public static double FLOWER_4_X = 134.0, FLOWER_4_Y = 100.0;
    /**
     * Half the side of the square kept clear around each flower centre (tower plus its scoring volume), inches.
     * FLOWER_HALF_IN and FLOWER_MARGIN_IN are both 0 NOW, which switches the flower keep-out areas OFF (the robot may
     * go anywhere near a flower; the tower positions above are still used to report how close it gets).
     * To turn them back on, set them to 10.0 and 4.0.
     */
    public static double FLOWER_HALF_IN = 0.0;
    /** Extra margin around that square, inches. */
    public static double FLOWER_MARGIN_IN = 0.0;

    /**
     * ASSUMED: HIVE structure half size around the field centre, inches (frame about 49 x 39 plus flowers), grown by the margin.
     * ALL THREE ARE 0 NOW, which switches the HIVE keep-out OFF. Leave and Park, Pollen Chase and DRIVEANDGRAB all use
     * these numbers, so none of them checks the HIVE until they are put back. To turn it back on, set them to 30.0, 30.0 and 6.0.
     */
    public static double HIVE_HALF_X_IN = 0.0;
    public static double HIVE_HALF_Y_IN = 0.0;
    public static double HIVE_MARGIN_IN = 0.0;

    /** Motor power cap while driving (0 to 1). */
    public static double MAX_POWER = 0.5;

    /** Wait this long after START before moving, for example to let an alliance partner clear the area. */
    public static double START_DELAY_SEC = 0.0;

    /** true = keep holding the end pose; false = let go once stopped. */
    public static boolean HOLD_END = true;

    /** Give up if the whole route takes longer than this, seconds. The supplied route is about 135 in; autonomous lasts 30 s. */
    public static double TIMEOUT_SEC = 25.0;

    /** true = drive the team's three-path route (Paths class); false = the original single move built from the zone. */
    public static boolean USE_SUPPLIED_ROUTE = true;

    /**
     * Supplied route only. true = refuse to start, and stop if it happens live, when the route leaves the field or breaks
     * a manual set-up rule (touching the wall, own alliance half, not in the zone, 18 in). The original route always
     * enforces everything.
     */
    public static boolean ENFORCE_FIELD_CHECKS = true;

    /**
     * Supplied route only. true = refuse to start, and stop live, if the route touches a HIVE or FLOWER keep-out area.
     * Off by default because those positions are estimates; the INIT screen warns instead and shows how close it gets.
     */
    public static boolean ENFORCE_KEEP_OUTS = false;

    /** Stop if the robot is more than this far from the line it is following, inches. */
    public static double MAX_OFF_LINE_IN = 6.0;

    static LeaveParkPlan buildPlan() { // package-private: PollenChase reuses the same field guard
        LeaveParkPlan.Config c = new LeaveParkPlan.Config();
        c.zoneXMin = ZONE_X_MIN;
        c.zoneYMin = ZONE_Y_MIN;
        c.zoneXMax = ZONE_X_MAX;
        c.zoneYMax = ZONE_Y_MAX;
        c.rotate180 = ROTATE_180;
        c.startSide = START_SIDE < 0 ? -1 : 1;
        c.robotLengthIn = ROBOT_LENGTH_IN;
        c.robotWidthIn = ROBOT_WIDTH_IN;
        c.startGapIn = START_GAP_IN;
        c.leaveClearanceIn = LEAVE_CLEARANCE_IN;
        c.hiveHalfXIn = HIVE_HALF_X_IN;
        c.hiveHalfYIn = HIVE_HALF_Y_IN;
        c.hiveMarginIn = HIVE_MARGIN_IN;
        c.flowers = new double[][]{
                {FLOWER_1_X, FLOWER_1_Y}, {FLOWER_2_X, FLOWER_2_Y}, {FLOWER_3_X, FLOWER_3_Y}, {FLOWER_4_X, FLOWER_4_Y}};
        c.flowerHalfIn = FLOWER_HALF_IN;
        c.flowerMarginIn = FLOWER_MARGIN_IN;
        return LeaveParkPlan.plan(c);
    }

    /**
     * The team's route, from the Pedro Pathing export, rewritten for Pedro 2.1.2. Same field names, same three paths,
     * same numbers. Differences from the export: Pose comes from com.pedropathing.geometry, PoseFactory.degrees().of(...)
     * is the pose(...) helper below, and Paths.line(a, b).linear(a, b) is a BezierLine with linear heading
     * interpolation. Each pathN() returns a PathChain, which is what Follower.followPath takes.
     */
    public static class Paths {
        private final Follower follower;
        private final boolean rotate180;

        public final Pose start;
        public final Pose path1;
        public final Pose point2Start;
        public final Pose point2;
        public final Pose point3Start;
        public final Pose point3;

        public Paths(Follower follower, boolean rotate180) {
            this.follower = follower;
            this.rotate180 = rotate180;
            start = pose(56, 8, 90);
            path1 = pose(23.3949, 23.9709, 90);
            point2Start = pose(23.3949, 23.9709, 90);
            point2 = pose(23.4597, 106.0436, 90);
            point3Start = pose(23.4597, 106.0436, 90);
            point3 = pose(11.415, 119.9653, 0);
        }

        /** Replaces PoseFactory.degrees().of(x, y, headingDegrees); also turns the pose 180 degrees for the other alliance. */
        private Pose pose(double x, double y, double headingDegrees) {
            if (rotate180) {
                return new Pose(LeaveParkPlan.FIELD_IN - x, LeaveParkPlan.FIELD_IN - y, Math.toRadians(headingDegrees + 180.0));
            }
            return new Pose(x, y, Math.toRadians(headingDegrees));
        }

        private PathChain line(Pose from, Pose to) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(from, to))
                    .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                    .build();
        }

        public PathChain path1() {
            return line(start, path1);
        }

        public PathChain path2() {
            return line(point2Start, point2);
        }

        public PathChain path3() {
            return line(point3Start, point3);
        }

        /** {from, to} of each path, in driving order. */
        public Pose[][] segments() {
            return new Pose[][]{{start, path1}, {point2Start, point2}, {point3Start, point3}};
        }

        public PathChain[] chains() {
            return new PathChain[]{path1(), path2(), path3()};
        }
    }

    /** What the checks found for one route: problems that stop the robot, warnings that only show, and clearances. */
    static final class Report {
        final List<String> blocking = new ArrayList<>();
        final List<String> warnings = new ArrayList<>();
        double[] closest;     // smallest gap to the HIVE, then to each flower tower, along the whole route (inches)
        double lengthIn;      // route length
        boolean leaveOk, parkOk;
        double endWallGap, endZoneOverlap;
        Pose[][] segments;
        boolean enforceField, enforceKeepOuts;
    }

    static Pose[][] generatedSegments(LeaveParkPlan plan) {
        return new Pose[][]{{new Pose(plan.startX, plan.startY, plan.heading), new Pose(plan.parkX, plan.parkY, plan.heading)}};
    }

    /** Runs every check on a route: walls, set-up rules, HIVE and FLOWER clearance, and whether it earns LEAVE and PARK. */
    static Report evaluate(LeaveParkPlan plan, Pose[][] segments, boolean supplied) {
        Report r = new Report();
        r.segments = segments;
        r.enforceField = !supplied || ENFORCE_FIELD_CHECKS;
        r.enforceKeepOuts = !supplied || ENFORCE_KEEP_OUTS;
        r.closest = null;

        List<String> wallHits = new ArrayList<>();
        List<String> keepOutHits = new ArrayList<>();
        for (int leg = 0; leg < segments.length; leg++) {
            Pose a = segments[leg][0], b = segments[leg][1];
            double len = Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
            r.lengthIn += len;
            int steps = Math.max(1, (int) Math.ceil(len / 1.0));
            for (int i = 0; i <= steps; i++) {
                double f = (double) i / steps;
                double x = a.getX() + (b.getX() - a.getX()) * f;
                double y = a.getY() + (b.getY() - a.getY()) * f;
                double h = a.getHeading() + (b.getHeading() - a.getHeading()) * f;
                String where = String.format(" (leg %d of %d, %.0f%% along it)", leg + 1, segments.length, f * 100);
                String w = plan.wallViolation(x, y, h);
                if (w != null && !containsReason(wallHits, w)) wallHits.add(w + where);
                String k = plan.keepOutViolation(x, y, h);
                if (k != null && !containsReason(keepOutHits, k)) keepOutHits.add(k + where);
                double[] clear = plan.clearances(x, y, h);
                if (r.closest == null) {
                    r.closest = clear.clone();
                } else {
                    for (int c = 0; c < clear.length; c++) r.closest[c] = Math.min(r.closest[c], clear[c]);
                }
            }
        }

        if (!supplied) {
            r.blocking.addAll(plan.problems); // the original route refuses on any problem, as before
        } else {
            Pose s0 = segments[0][0];
            for (String p : plan.startRuleProblems(s0.getX(), s0.getY(), s0.getHeading())) {
                (r.enforceField ? r.blocking : r.warnings).add(p);
            }
            (r.enforceField ? r.blocking : r.warnings).addAll(wallHits);
            for (String k : keepOutHits) {
                (r.enforceKeepOuts ? r.blocking : r.warnings).add(k + (r.enforceKeepOuts ? "" : " [warning only]"));
            }
        }

        Pose end = segments[segments.length - 1][1];
        r.endWallGap = plan.wallGap(end.getX(), end.getY(), end.getHeading());
        r.endZoneOverlap = plan.overlapWithZone(end.getX(), end.getY(), end.getHeading());
        r.leaveOk = r.endWallGap > 0.5;
        r.parkOk = r.endZoneOverlap > 0;
        if (!r.leaveOk) r.warnings.add("End pose is still touching the wall: no LEAVE.");
        if (!r.parkOk) r.warnings.add("End pose is not in the LOADING ZONE as set here: no PARK.");
        return r;
    }

    private static boolean containsReason(List<String> hits, String reason) {
        for (String h : hits) if (h.startsWith(reason)) return true;
        return false;
    }

    static void showReport(Telemetry telemetry, Report r, boolean supplied) {
        boolean ok = r.blocking.isEmpty();
        telemetry.addData("route", supplied ? "supplied 3-path route" + (ROTATE_180 ? " (turned 180: blue)" : " (red)")
                : "original zone route");
        telemetry.addData("checks", ok ? "OK, will run" : "REFUSED (robot will not move)");
        for (String p : r.blocking) telemetry.addLine("  - " + p);
        for (String w : r.warnings) telemetry.addLine("  ! " + w);
        for (int i = 0; i < r.segments.length; i++) {
            Pose a = r.segments[i][0], b = r.segments[i][1];
            telemetry.addData("leg " + (i + 1), "(%.1f, %.1f, %.0f) -> (%.1f, %.1f, %.0f)", a.getX(), a.getY(),
                    Math.toDegrees(a.getHeading()), b.getX(), b.getY(), Math.toDegrees(b.getHeading()));
        }
        telemetry.addData("route length (in)", "%.0f", r.lengthIn);
        telemetry.addData("LEAVE / PARK at the end", "%s (gap to wall %.1f in) / %s (%.0f sq in in zone)",
                r.leaveOk ? "yes" : "NO", r.endWallGap, r.parkOk ? "yes" : "NO", r.endZoneOverlap);
        if (r.closest != null) {
            StringBuilder sb = new StringBuilder(Double.isInfinite(r.closest[0]) ? "HIVE off" : String.format("HIVE %.1f", r.closest[0]));
            for (int i = 1; i < r.closest.length; i++) sb.append(String.format(", F%d %.1f", i, r.closest[i]));
            telemetry.addData("closest the robot gets to each element (in)", sb.toString());
        }
        telemetry.addData("keep-outs enforced", r.enforceKeepOuts ? "yes" : "no (warnings only; positions are estimates)");
        telemetry.addLine("SET-UP: touching the wall, fully on your own half, not in the zone, inside 18 in,"
                + " touching exactly 4 POLLEN (none in the robot's path), then stay still.");
    }

    /** Distance from a point to the straight line a to b (clamped to its ends). */
    static double offLine(Pose a, Pose b, double x, double y) {
        double dx = b.getX() - a.getX(), dy = b.getY() - a.getY();
        double len2 = dx * dx + dy * dy;
        double f = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((x - a.getX()) * dx + (y - a.getY()) * dy) / len2));
        return Math.hypot(x - (a.getX() + f * dx), y - (a.getY() + f * dy));
    }

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Drawing.init();

        // Show the checks while waiting, and refresh them so a value changed in Panels shows up straight away.
        while (opModeInInit()) {
            LeaveParkPlan plan = buildPlan();
            boolean supplied = USE_SUPPLIED_ROUTE;
            Pose[][] segs = supplied ? new Paths(follower, ROTATE_180).segments() : generatedSegments(plan);
            showReport(telemetry, evaluate(plan, segs, supplied), supplied);
            telemetry.update();
            sleep(100);
        }
        if (isStopRequested()) return;

        // Everything again after START so the latest Panels values are used.
        final LeaveParkPlan plan = buildPlan();
        final boolean supplied = USE_SUPPLIED_ROUTE;
        Paths paths = new Paths(follower, ROTATE_180);
        Pose[][] segments = supplied ? paths.segments() : generatedSegments(plan);
        Report report = evaluate(plan, segments, supplied);
        if (!report.blocking.isEmpty()) {
            while (opModeIsActive()) { // do nothing, just explain
                showReport(telemetry, report, supplied);
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
            // One straight line, heading held constant: no turning, so the footprint never rotates into anything.
            chains = new PathChain[]{follower.pathBuilder()
                    .addPath(new BezierLine(segments[0][0], segments[0][1]))
                    .setConstantHeadingInterpolation(plan.heading)
                    .build()};
        }

        double power = Math.max(0.1, Math.min(1.0, MAX_POWER));
        int leg = 0;
        boolean started = false;
        boolean finished = false;
        boolean released = false;
        String aborted = null;
        ElapsedTime clock = new ElapsedTime();

        try {
            while (opModeIsActive()) {
                follower.update();
                Pose pose = follower.getPose();

                if (!started && clock.seconds() >= START_DELAY_SEC) {
                    // Only the last leg holds its end pose; the others just run on into the next.
                    follower.followPath(chains[0], power, chains.length == 1 && HOLD_END);
                    started = true;
                    clock.reset();
                } else if (started && !finished && aborted == null && !follower.isBusy()) {
                    leg++;
                    if (leg >= chains.length) {
                        finished = true;
                    } else {
                        follower.followPath(chains[leg], power, leg == chains.length - 1 && HOLD_END);
                    }
                }

                if (started && !finished && aborted == null) {
                    // Live safety net: stop if the robot is where the checks say it must never be, or has wandered.
                    String reason = null;
                    String wall = plan.wallViolation(pose.getX(), pose.getY(), pose.getHeading());
                    if (wall != null && report.enforceField) reason = wall;
                    String keep = plan.keepOutViolation(pose.getX(), pose.getY(), pose.getHeading());
                    if (reason == null && keep != null && report.enforceKeepOuts) reason = keep;
                    if (reason == null && offLine(segments[leg][0], segments[leg][1], pose.getX(), pose.getY()) > MAX_OFF_LINE_IN) {
                        reason = "robot is more than " + MAX_OFF_LINE_IN + " in off the line it is following (leg " + (leg + 1) + ")";
                    }
                    if (reason == null && clock.seconds() > TIMEOUT_SEC) {
                        reason = "route took longer than " + TIMEOUT_SEC + " s";
                    }
                    if (reason != null) {
                        aborted = reason;
                        follower.breakFollowing(); // motors off
                    }
                }

                if (finished && aborted == null && !HOLD_END && !released) {
                    follower.breakFollowing(); // already stopped: release the motors, once
                    released = true;
                }

                telemetry.addData("state", aborted != null ? "ABORTED: " + aborted
                        : !started ? "waiting for start delay"
                        : finished ? "done" : "driving leg " + (leg + 1) + " of " + chains.length);
                telemetry.addData("x", "%.1f", pose.getX());
                telemetry.addData("y", "%.1f", pose.getY());
                telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.getHeading()));
                // What the wheels say, not what the referee sees: check the real robot against the tape.
                telemetry.addData("LEAVE by odometry (gap to wall, in)", "%.1f", plan.wallGap(pose.getX(), pose.getY(), pose.getHeading()));
                telemetry.addData("PARK by odometry (area in zone, sq in)", "%.0f", plan.overlapWithZone(pose.getX(), pose.getY(), pose.getHeading()));
                telemetry.update();
                Drawing.drawDebug(follower);
            }
        } finally {
            follower.setMaxPower(1.0); // the follower settings are shared with the other OpModes
        }
    }
}
