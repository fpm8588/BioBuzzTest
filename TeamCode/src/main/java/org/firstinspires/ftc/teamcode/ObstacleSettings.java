package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;
import org.firstinspires.ftc.teamcode.util.ObstacleMath;
import org.firstinspires.ftc.teamcode.util.ObstacleTracker;
import org.firstinspires.ftc.teamcode.util.PollenMath;

import java.util.ArrayList;
import java.util.List;

/*
 * Every number the obstacle (opposing robot) avoidance depends on, shared by AvoidAndPark and Obstacle Vision Test.
 * Change them in Panels (Configurables > ObstacleSettings), or here and rebuild.
 *
 * The camera sees the opposing alliance's colour (every robot must carry alliance-coloured ROBOT SIGNS, rule R402),
 * turns how big that patch looks into a distance, and puts an obstacle on the field map. The calibration steps that
 * produce DISTANCE_K, CAMERA_FORWARD_OFFSET_IN, MIN_AREA and the radii are in docs/LIMELIGHT_OBSTACLES.md.
 */
@Configurable
public class ObstacleSettings {

    // ---- on / off ----

    /** false = AvoidAndPark just drives the route, the camera is not used. */
    public static boolean ENABLE_AVOIDANCE = true;

    /**
     * true = refuse to move if the obstacle pipeline cannot be confirmed (the safe default: a robot that quietly drives with no
     * obstacle sensing is the failure this exists to prevent); false = drive the route without avoidance.
     */
    public static boolean REQUIRE_VISION = true;

    // ---- Limelight pipelines (one slot per colour; the camera runs one pipeline at a time) ----

    /** Slot holding the RED robot-sign colour pipeline. Used when WE are blue (LeaveAndPark.ROTATE_180 = true). */
    public static int PIPELINE_RED_ROBOTS = 1;
    /** Slot holding the BLUE robot-sign colour pipeline. Used when WE are red (LeaveAndPark.ROTATE_180 = false). */
    public static int PIPELINE_BLUE_ROBOTS = 2;

    /**
     * Pipeline slot to use instead of the one chosen from the alliance. -1 = automatic. Set it to match the COLOUR OF THE SIGN ON THE
     * ROBOT YOU ARE TESTING AGAINST (a dummy robot is often the wrong colour for your alliance's opponents).
     */
    public static int PIPELINE_OVERRIDE = -1;

    /** Ignore results older than this many milliseconds. */
    public static long MAX_STALENESS_MS = 100;

    // ---- seeing: size of the patch -> distance ----

    /** Ignore patches smaller than this fraction of the image (0 to 1). Sets how far away an obstacle can be seen. */
    public static double MIN_AREA = 0.001;

    /**
     * distance (in) = DISTANCE_K / sqrt(area). PLACEHOLDER. Measure it: put a robot sign / bumper the way it will be seen at a
     * known distance and let Obstacle Vision Test suggest the value (calibration step 8). Different robots have different sign
     * sizes, so the result is only good to about +/- 30 percent.
     */
    public static double DISTANCE_K = 5.0;

    /**
     * Height of the centre of the coloured robot sign above the floor, inches. PLACEHOLDER: measure real robots' signs. Only used
     * to warn (in Obstacle Vision Test) when the camera is tilted down so far that signs this high are out of the picture
     * beyond a short range. The camera mounting numbers (height, pitch, field of view) are the ones in PollenSettings.
     */
    public static double TYPICAL_SIGN_HEIGHT_IN = 6.0;

    /** How far in front of the robot's centre the camera lens is, inches. PLACEHOLDER: measure it. */
    public static double CAMERA_FORWARD_OFFSET_IN = 6.0;

    /** true if tx has the opposite sign (camera mounted so that right is not right). Check in Obstacle Vision Test. */
    public static boolean INVERT_TX = false;

    // ---- remembering ----

    /** A new sighting this close to a remembered obstacle is the same obstacle, inches. */
    public static double MATCH_RADIUS_IN = 14.0;
    /** An obstacle must be seen for this long, and at least twice, before the robot reacts, seconds. */
    public static double CONFIRM_TIME_SEC = 0.10;
    /**
     * An obstacle the camera COULD see (it is inside the picture) but does not is forgotten after this long, seconds. This is also
     * what absorbs a flickering pipeline: a blob that comes and goes keeps the obstacle alive.
     */
    public static double MEMORY_SEC = 1.5;
    /**
     * An obstacle the camera CANNOT see (beside the robot, inside the blind zone just in front of the lens, too far, off to the
     * side) is kept this long, seconds. NOT SEEING AN OBSTACLE IS NOT THE SAME AS IT BEING GONE: a robot stopped right beside
     * one cannot see it and must not drive into it.
     */
    public static double HARD_MEMORY_SEC = 8.0;
    /** A sighting this close to the robot counts at once (no confirmation delay), inches. A false stop is cheap; a miss is not. 0 = off. */
    public static double IMMEDIATE_RANGE_IN = 60.0;
    /** How wide the camera sees, degrees. 54.5 is the Limelight 3A figure as I remember it: check it against the spec sheet. */
    public static double CAMERA_HORIZONTAL_FOV_DEG = 54.5;

    // ---- how much room to give ----

    /** Radius assumed for every obstacle, inches (an opposing robot is up to about 18 in across, so 12 leaves slack). */
    public static double OBSTACLE_RADIUS_IN = 12.0;
    /** Our robot's half-width plus a little (16 in wide = 8, plus 2). Distance from our centreline to the nearest corner. */
    public static double ROBOT_CLEARANCE_RADIUS_IN = 10.0;
    /** Extra room on top of the two radii, inches. */
    public static double CLEARANCE_MARGIN_IN = 3.0;
    /**
     * A path only counts as blocked if it comes more than this far INSIDE the clearance, inches. Distances are estimates, and a
     * detour sits exactly on the clearance, so without this the robot would flag its own detour as blocked.
     */
    public static double BLOCK_SLACK_IN = 3.0;
    /** Only react to obstacles this far ahead along the path, inches. */
    public static double TRIGGER_RANGE_IN = 45.0;
    /**
     * Stop at once if an obstacle ahead gets closer than its radius plus ours plus this, inches. Keep it BELOW CLEARANCE_MARGIN_IN
     * minus BLOCK_SLACK_IN (here 0 is below 3 - 3), or the robot's own detour, which runs at the full clearance, trips it.
     */
    public static double EMERGENCY_MARGIN_IN = 0.0;

    // ---- how it reacts ----

    /** How often the path is checked against the obstacles, seconds. */
    public static double REPLAN_INTERVAL_SEC = 0.1;
    /** If there is no way round, wait this long for the obstacle to move, then give up and stop, seconds. */
    public static double BLOCKED_WAIT_SEC = 4.0;
    /** Give up after this many detours in one run. */
    public static int MAX_DETOURS = 3;

    /**
     * true = when there is no room beside the obstacle, find ANY legal way to the goal, even the long way round the far side of the
     * HIVE. The path is planned around the HIVE and flowers at the PLAN_ sizes below, so MEASURE THEM first (they are estimates).
     * false = stop and wait instead.
     */
    public static boolean ALLOW_LONG_WAY_ROUND = true;
    /** Longest acceptable long way, as extra distance over going straight, inches. */
    public static double LONG_WAY_MAX_EXTRA_IN = 260.0;
    /** Length of autonomous, seconds. The long way is only taken if it can finish well inside this. */
    public static double AUTO_LENGTH_SEC = 30.0;

    // ---- field elements, for planning only (separate from the keep-out switches in LeaveAndPark, which are off) ----

    /**
     * Detours are planned around the HIVE and the flowers using these sizes, so a detour never swings into them. The flower
     * numbers cover only the tower itself (radius about 3.5 in, so half 4.5), because the Leave and Park route already runs
     * within a few inches of flower 2 and a bigger area would make every detour near it look illegal. Positions are estimates:
     * measure them (LeaveAndPark FLOWER_n_X / FLOWER_n_Y).
     */
    public static double PLAN_HIVE_HALF_IN = 25.0;
    public static double PLAN_HIVE_MARGIN_IN = 3.0;
    public static double PLAN_FLOWER_HALF_IN = 4.5;
    public static double PLAN_FLOWER_MARGIN_IN = 0.0;

    /**
     * true = sightings that land inside the HIVE or a flower area are thrown away. The HIVE and NECTAR carry red and blue, so
     * they would otherwise look like robots.
     */
    public static boolean IGNORE_STATIC_ELEMENTS = true;

    // ---- helpers ----

    /** Pipeline slot for the OPPOSING alliance's colour: red alliance (ROTATE_180 false) faces blue robots. */
    static int opponentPipeline() {
        if (PIPELINE_OVERRIDE >= 0) return PIPELINE_OVERRIDE;
        return LeaveAndPark.ROTATE_180 ? PIPELINE_RED_ROBOTS : PIPELINE_BLUE_ROBOTS;
    }

    /** Nearest and farthest distance, from the lens, at which a robot sign at TYPICAL_SIGN_HEIGHT_IN is in the picture. */
    static double[] visibleRange() {
        double near = PollenMath.nearestVisibleDistanceIn(PollenSettings.CAMERA_HEIGHT_IN, PollenSettings.CAMERA_PITCH_DEG,
                PollenSettings.CAMERA_VERTICAL_FOV_DEG, TYPICAL_SIGN_HEIGHT_IN);
        double far = ObstacleMath.farthestVisibleDistanceIn(PollenSettings.CAMERA_HEIGHT_IN, PollenSettings.CAMERA_PITCH_DEG,
                PollenSettings.CAMERA_VERTICAL_FOV_DEG, TYPICAL_SIGN_HEIGHT_IN);
        return new double[]{Double.isNaN(near) ? 0 : near, Double.isNaN(far) ? 1e9 : far};
    }

    /** The field guard used for planning: walls plus the HIVE and flower areas at the PLAN_ sizes, our robot's size. */
    static LeaveParkPlan planningGuard() {
        LeaveParkPlan.Config c = new LeaveParkPlan.Config();
        c.zoneXMin = LeaveAndPark.ZONE_X_MIN;
        c.zoneYMin = LeaveAndPark.ZONE_Y_MIN;
        c.zoneXMax = LeaveAndPark.ZONE_X_MAX;
        c.zoneYMax = LeaveAndPark.ZONE_Y_MAX;
        c.rotate180 = LeaveAndPark.ROTATE_180;
        c.robotLengthIn = LeaveAndPark.ROBOT_LENGTH_IN;
        c.robotWidthIn = LeaveAndPark.ROBOT_WIDTH_IN;
        c.hiveHalfXIn = PLAN_HIVE_HALF_IN;
        c.hiveHalfYIn = PLAN_HIVE_HALF_IN;
        c.hiveMarginIn = PLAN_HIVE_MARGIN_IN;
        c.flowers = new double[][]{
                {LeaveAndPark.FLOWER_1_X, LeaveAndPark.FLOWER_1_Y}, {LeaveAndPark.FLOWER_2_X, LeaveAndPark.FLOWER_2_Y},
                {LeaveAndPark.FLOWER_3_X, LeaveAndPark.FLOWER_3_Y}, {LeaveAndPark.FLOWER_4_X, LeaveAndPark.FLOWER_4_Y}};
        c.flowerHalfIn = PLAN_FLOWER_HALF_IN;
        c.flowerMarginIn = PLAN_FLOWER_MARGIN_IN;
        return LeaveParkPlan.plan(c);
    }

    /** A fresh tracker whose ignore areas are the planning HIVE and flower areas. */
    static ObstacleTracker newTracker(LeaveParkPlan guard) {
        List<double[]> ignore = new ArrayList<>();
        if (IGNORE_STATIC_ELEMENTS) {
            if (guard.hiveEnabled()) ignore.add(guard.keepOut.clone());
            if (guard.flowersEnabled()) ignore.addAll(guard.flowerKeepOuts);
        }
        return new ObstacleTracker(MATCH_RADIUS_IN, CONFIRM_TIME_SEC, MEMORY_SEC, HARD_MEMORY_SEC, OBSTACLE_RADIUS_IN, IMMEDIATE_RANGE_IN, ignore);
    }

    /** Field position of one blob seen from a robot pose, or null if the blob is too small or has no distance. */
    static double[] position(Pose pose, PollenMath.Blob b) {
        if (b.area < MIN_AREA) return null;
        double dist = ObstacleMath.distanceFromArea(b.area, DISTANCE_K);
        if (Double.isNaN(dist)) return null;
        double tx = INVERT_TX ? -b.txDeg : b.txDeg;
        return ObstacleMath.fieldPosition(pose.getX(), pose.getY(), pose.getHeading(), tx, dist, CAMERA_FORWARD_OFFSET_IN);
    }
}
