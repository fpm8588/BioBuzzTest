package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;

/*
 * BIOBUZZ autonomous: earns LEAVE (3) and AUTO PARK (5) and nothing else.
 *
 *   LEAVE = the robot is no longer touching the perimeter wall at the end of autonomous.
 *   PARK  = the robot is at least partially in its own LOADING ZONE at the end of autonomous.
 *
 * The robot starts with its back edge against the wall, just beside its LOADING ZONE, and drives one short straight
 * line (about 23 inches, no turning) that backs it off the wall and into the zone. The start and end poses are
 * worked out from the zone rectangle by util/LeaveParkPlan, so the only things to check on the real field are the
 * numbers below.
 *
 * Field elements and how they are avoided:
 *   - Perimeter walls: the plan keeps the robot's whole footprint inside the field, and the route ends clear of
 *     the wall by LEAVE_CLEARANCE_IN.
 *   - HIVE structure (with its flowers) in the middle of the field: a keep-out square around it is checked along the
 *     whole planned line before the robot moves, and against the live position every loop while it moves. The route
 *     stays at the field edge, more than 35 in from the keep-out.
 *   - Other robots can not be planned for. Agree with your alliance partner which of you leaves first, and use
 *     START_DELAY_SEC if you need to wait.
 * If any check fails, the robot does not move and the screen says why. If the robot ever leaves the plan, it stops.
 *
 * Robot set-up rules (manual, "ROBOTS must be set up correctly on the FIELD") and how this OpMode handles each:
 *   A. fully on its own alliance side (columns A, B, C = red, D, E, F = blue): checked by the plan. The side comes
 *      from where the loading zone is, and the plan refuses a start that is not fully inside that half.
 *   B. not attached to or hanging from a field element: you check this by eye.
 *   C. touching the perimeter wall: the start pose puts the back edge exactly on the wall.
 *   D. not touching or inside a FLOWER's scoring volume: the flowers are part of the HIVE structure in the middle,
 *      far from the start. Check by eye; the plan also keeps clear of a keep-out area around the HIVE.
 *   E. not in the LOADING ZONE: the start is beside the zone with START_GAP_IN (at least 1 in) of space.
 *   F. inside the starting configuration (18 in cube): the plan refuses a robot size over 18 in. Height and any
 *      parts that unfold are for you to check.
 *   G. in contact with exactly 4 POLLEN pre-loads, on your own side: you do this. Do not put a pre-load between the
 *      robot and the zone or in the robot's path (the robot drives away from the wall, toward the zone side).
 *   H. fully motionless after INIT: the only motor command before START is zero power (from building the follower).
 *
 * VERIFY BEFORE USE (see the numbers marked ASSUMED): the game manual could not be read from the build machine, so
 * the loading zone position and the HIVE size came from search results, not the manual.
 * 1. Measure the LOADING ZONE tape on your field and fix ZONE_* (inches from the bottom-left corner of your frame).
 *    Per the manual's columns, the red zone is in column A and the blue zone in column F.
 * 2. Set ROBOT_LENGTH_IN / ROBOT_WIDTH_IN to the real chassis size in the start configuration (at most 18 x 18).
 *
 * Setup: robot centred beside the zone as shown on the Driver Station, back edge flat against the wall, facing away
 * from the wall. The Driver Station prints the exact start and end poses at INIT.
 * All numbers can be changed in Panels (Configurables > LeaveAndPark) during INIT.
 */
@Configurable
@Autonomous(name = "Leave and Park", group = "Competition")
public class LeaveAndPark extends LinearOpMode {

    // ASSUMED: the red LOADING ZONE is on tile A5 and the blue one on tile F2 (a 180 degree turn of each other),
    // about 23 in along the wall and 11 in deep. Here A5 is taken as the left wall, 5th tile up from the bottom.
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

    /** Which side of the zone to start on along the wall: -1 or +1. Pick the side with more free space. */
    public static int START_SIDE = -1;

    /** Real robot size in the start configuration, inches (18 x 18 is the largest allowed to start). */
    public static double ROBOT_LENGTH_IN = 18.0; // front to back
    public static double ROBOT_WIDTH_IN = 18.0;  // side to side

    /** Gap between the robot and the zone edge at the start, inches. At least 1: the robot must not be in the zone. */
    public static double START_GAP_IN = 2.0;

    /** How far the robot's back edge ends up from the wall. Must be above 0 for LEAVE; zone depth minus this is the overlap. */
    public static double LEAVE_CLEARANCE_IN = 5.0;

    /** ASSUMED: HIVE structure half size around the field centre, inches (frame about 49 x 39 plus flowers), grown by the margin. */
    public static double HIVE_HALF_X_IN = 30.0;
    public static double HIVE_HALF_Y_IN = 30.0;
    public static double HIVE_MARGIN_IN = 6.0;

    /** Motor power cap while driving (0 to 1). */
    public static double MAX_POWER = 0.5;

    /** Wait this long after START before moving, for example to let an alliance partner clear the area. */
    public static double START_DELAY_SEC = 0.0;

    /** true = keep holding the end pose; false = let go once stopped. */
    public static boolean HOLD_END = true;

    /** Give up if the move takes longer than this, seconds. A 23 in move takes about 2 s. */
    public static double TIMEOUT_SEC = 8.0;

    /** Stop if the robot is more than this far from the planned line, inches. */
    public static double MAX_OFF_LINE_IN = 6.0;

    private static LeaveParkPlan buildPlan() {
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
        return LeaveParkPlan.plan(c);
    }

    private void showPlan(LeaveParkPlan plan) {
        telemetry.addData("plan", plan.ok() ? "OK" : "REFUSED (robot will not move)");
        for (String problem : plan.problems) telemetry.addLine("  - " + problem);
        telemetry.addData("zone (x0,y0,x1,y1)", "%.1f, %.1f, %.1f, %.1f", plan.zone[0], plan.zone[1], plan.zone[2], plan.zone[3]);
        telemetry.addData("START centre (x, y, heading)", "%.1f, %.1f, %.0f deg", plan.startX, plan.startY, Math.toDegrees(plan.heading));
        telemetry.addData("PARK  centre (x, y, heading)", "%.1f, %.1f, %.0f deg", plan.parkX, plan.parkY, Math.toDegrees(plan.heading));
        telemetry.addData("move length (in)", "%.1f", Math.hypot(plan.parkX - plan.startX, plan.parkY - plan.startY));
        telemetry.addData("alliance half (start must be fully inside)", plan.allianceHalf);
        telemetry.addLine("SET-UP: back edge flat on the wall, beside the zone, facing away from the wall, fully on your own half,"
                + " not in the zone, inside 18 in, touching exactly 4 POLLEN (none between robot and zone), then stay still.");
    }

    /** Distance from a point to the planned straight line (clamped to its ends). */
    private static double offLine(LeaveParkPlan p, double x, double y) {
        double dx = p.parkX - p.startX, dy = p.parkY - p.startY;
        double len2 = dx * dx + dy * dy;
        double f = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((x - p.startX) * dx + (y - p.startY) * dy) / len2));
        return Math.hypot(x - (p.startX + f * dx), y - (p.startY + f * dy));
    }

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Drawing.init();

        // Show the plan while waiting, and refresh it so a value changed in Panels shows up straight away.
        while (opModeInInit()) {
            showPlan(buildPlan());
            telemetry.update();
            sleep(100);
        }
        if (isStopRequested()) return;

        // Plan again after START so the latest Panels values are used.
        LeaveParkPlan plan = buildPlan();
        if (!plan.ok()) {
            while (opModeIsActive()) { // do nothing, just explain
                showPlan(plan);
                telemetry.update();
                sleep(200);
            }
            return;
        }

        Pose start = new Pose(plan.startX, plan.startY, plan.heading);
        Pose park = new Pose(plan.parkX, plan.parkY, plan.heading);
        follower.setStartingPose(start);

        // One straight line, heading held constant: no turning, so the footprint never rotates into anything.
        PathChain move = follower.pathBuilder()
                .addPath(new BezierLine(start, park))
                .setConstantHeadingInterpolation(plan.heading)
                .build();

        double power = Math.max(0.1, Math.min(1.0, MAX_POWER));
        boolean started = false;
        boolean released = false;
        String aborted = null;
        ElapsedTime clock = new ElapsedTime();

        try {
            while (opModeIsActive()) {
                follower.update();
                Pose pose = follower.getPose();

                if (!started && clock.seconds() >= START_DELAY_SEC) {
                    follower.followPath(move, power, HOLD_END);
                    started = true;
                    clock.reset();
                }

                if (started && aborted == null) {
                    // Live safety net: stop if the robot is where the plan says it must never be, or has wandered.
                    String reason = plan.violation(pose.getX(), pose.getY(), pose.getHeading());
                    if (reason == null && offLine(plan, pose.getX(), pose.getY()) > MAX_OFF_LINE_IN) {
                        reason = "robot is more than " + MAX_OFF_LINE_IN + " in off the planned line";
                    }
                    if (reason == null && follower.isBusy() && clock.seconds() > TIMEOUT_SEC) {
                        reason = "move took longer than " + TIMEOUT_SEC + " s";
                    }
                    if (reason != null) {
                        aborted = reason;
                        follower.breakFollowing(); // motors off
                    }
                }

                if (started && aborted == null && !follower.isBusy() && !HOLD_END && !released) {
                    follower.breakFollowing(); // already stopped: release the motors, once
                    released = true;
                }

                telemetry.addData("state", aborted != null ? "ABORTED: " + aborted
                        : !started ? "waiting for start delay" : follower.isBusy() ? "driving" : "done");
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
