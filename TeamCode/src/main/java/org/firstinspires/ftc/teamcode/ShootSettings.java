package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;
import org.firstinspires.ftc.teamcode.util.ShootPlan;

/*
 * Every number the "Shoot On Tags" autonomous and its test depend on, in one place. Change them in Panels (Configurables > ShootSettings),
 * or here and rebuild. Everything marked ASSUMED is read off a drawing or an earlier estimate: measure it on the real field.
 *
 * Everything here is for the RED alliance; the BLUE alliance's plan is the same turned 180 degrees about the field centre, chosen by
 * LeaveAndPark.ROTATE_180 like every other autonomous.
 */
@Configurable
public class ShootSettings {

    // ---- The HIVE (ASSUMED: the frame on the floor, about 49 x 39 in, centred on the field centre; from LeaveAndPark / the field drawing) ----

    public static double HIVE_CENTRE_X_IN = 72.0;
    public static double HIVE_CENTRE_Y_IN = 72.0;
    /** Frame size along x (red wall to blue wall) and along y, inches. */
    public static double HIVE_SIZE_X_IN = 49.0;
    public static double HIVE_SIZE_Y_IN = 39.0;

    /** The route stays at least this far from the HIVE frame (plus the robot's own size), inches. */
    public static double HIVE_ROUTE_MARGIN_IN = 6.0;

    // ---- Where the tag strip is (ASSUMED from the HIVE drawing: the strips hang under the CELLs, red on the red half, blue on the blue half) ----

    /**
     * The face of the HIVE the RED strip is on: 0 = toward the red wall (-x), 1 = toward the blue wall (+x), 2 = toward the bottom wall (-y),
     * 3 = toward the top wall (+y). The default is 2: the red robots start on the bottom wall, in the red (x below 72) half, and the red cells
     * are on the red half. Check on the field with Shoot On Tags Test.
     */
    public static int STRIP_FACE = ShootPlan.MINUS_Y;

    /**
     * Where the strip's centre is along that face, inches from the face's centre (+ = toward +y on an x face, toward +x on a y face).
     * The default -12.25 is the middle of the red half of a 49 in face (a quarter of its width toward the red wall).
     */
    public static double STRIP_ALONG_OFFSET_IN = -12.25;

    // ---- Where the robot shoots from ----

    /**
     * Distance from the CAMERA to the plane of the strip, measured straight out from the HIVE face, inches. ASSUMED: tune it for your shooter.
     * Shoot On Tags Test shows the least and most this can be on this field.
     */
    public static double SHOOT_DISTANCE_IN = 30.0;

    // ---- Start (red frame; turned 180 degrees for blue). Same start as the other autonomous routes. ----

    public static double START_X_IN = 56.0;
    public static double START_Y_IN = 8.0;
    public static double START_HEADING_DEG = 90.0;

    // ---- Driving ----

    public static double MAX_POWER = 0.5;
    public static double START_DELAY_SEC = 0.0;
    /** Give up on the whole run after this long, seconds. Autonomous lasts 30. */
    public static double TIMEOUT_SEC = 25.0;
    /** Stop if the robot is more than this far from the line it is following, inches. */
    public static double MAX_OFF_PATH_IN = 6.0;

    // ---- Fine positioning with the camera (only if a strip tag is in view) ----

    /** true = after arriving, turn to face the strip and creep to SHOOT_DISTANCE_IN by the Limelight's range, if a tag of the strip is in view. */
    public static boolean USE_VISION = true;
    /** Never let vision move the robot more than this far from the planned shooting position, inches. */
    public static double MAX_CORRECTION_IN = 8.0;
    /** Range is good enough within this many inches of SHOOT_DISTANCE_IN. */
    public static double RANGE_TOLERANCE_IN = 1.5;
    /** Forward power per inch of range error, and its limit. */
    public static double K_RANGE = 0.03;
    public static double MAX_CORRECT_POWER = 0.3;
    /** Give up the fine positioning after this long, seconds. */
    public static double VISION_TIMEOUT_SEC = 2.0;

    // ---- Shooting ----

    /** Time from the shooter starting to spin until the intake feeds, seconds (the shooter starts spinning when the robot starts driving). */
    public static double SPINUP_SEC = 1.5;
    /** How long the intake feeds, seconds. */
    public static double FEED_SEC = 2.0;
    /** true = the far-range velocity preset (Shooter.HIGH_VELOCITY), false = the close-range one (LOW_VELOCITY). */
    public static boolean USE_HIGH_SHOT = false;
    /** Shot-angle adjustor position (0 to 1) for this shot. */
    public static double ADJUSTOR_POSITION = 0.5;

    // ---- For the warnings only ----

    /**
     * ASSUMED: height of the strip's tags above the floor, inches, used for the INIT warnings until AprilTagSettings.TAG_CENTRE_HEIGHT_IN is set.
     * Read off the HIVE drawing against its 49 in width: the lower cluster is about 37 in up, the upper about 51 in (plus or minus 20 percent).
     */
    public static double TAG_HEIGHT_ESTIMATE_IN = 37.0;

    // ---- Built from the numbers above ----

    public static ShootPlan.Config planConfig() {
        ShootPlan.Config c = new ShootPlan.Config();
        c.hiveCx = HIVE_CENTRE_X_IN;
        c.hiveCy = HIVE_CENTRE_Y_IN;
        c.hiveSizeX = HIVE_SIZE_X_IN;
        c.hiveSizeY = HIVE_SIZE_Y_IN;
        c.face = STRIP_FACE;
        c.alongOffsetIn = STRIP_ALONG_OFFSET_IN;
        c.standoffIn = SHOOT_DISTANCE_IN;
        c.cameraForwardIn = ObstacleSettings.CAMERA_FORWARD_OFFSET_IN;
        c.robotLengthIn = LeaveAndPark.ROBOT_LENGTH_IN;
        c.robotWidthIn = LeaveAndPark.ROBOT_WIDTH_IN;
        c.rotate180 = LeaveAndPark.ROTATE_180;
        return c;
    }

    /** The robot's start pose for the chosen alliance. */
    public static Pose startPose() {
        if (LeaveAndPark.ROTATE_180) {
            return new Pose(ShootPlan.FIELD_IN - START_X_IN, ShootPlan.FIELD_IN - START_Y_IN, Math.toRadians(START_HEADING_DEG + 180.0));
        }
        return new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));
    }

    /**
     * The field guard for this autonomous: the walls and a keep-out rectangle round the HIVE frame (the frame's size plus the route margin).
     * The flowers are left out (their positions are estimates and their keep-outs are switched off elsewhere).
     */
    static LeaveParkPlan guard() {
        LeaveParkPlan.Config c = new LeaveParkPlan.Config();
        c.zoneXMin = LeaveAndPark.ZONE_X_MIN;
        c.zoneYMin = LeaveAndPark.ZONE_Y_MIN;
        c.zoneXMax = LeaveAndPark.ZONE_X_MAX;
        c.zoneYMax = LeaveAndPark.ZONE_Y_MAX;
        c.rotate180 = LeaveAndPark.ROTATE_180;
        c.robotLengthIn = LeaveAndPark.ROBOT_LENGTH_IN;
        c.robotWidthIn = LeaveAndPark.ROBOT_WIDTH_IN;
        c.hiveHalfXIn = HIVE_SIZE_X_IN / 2;
        c.hiveHalfYIn = HIVE_SIZE_Y_IN / 2;
        c.hiveMarginIn = HIVE_ROUTE_MARGIN_IN;
        c.flowerHalfIn = 0;
        c.flowerMarginIn = 0;
        return LeaveParkPlan.plan(c);
    }
}
