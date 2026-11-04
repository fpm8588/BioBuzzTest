package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.TagMath;

/*
 * Every number the goal-tag aiming and ranging depends on, in one place, shared by GoalTagTest, CompTeleOp and CompAuto.
 * Change them in Panels (Configurables > AprilTagSettings), or here and rebuild.
 *
 * The camera itself (height above the floor, tilt, lens angle) is the one in PollenSettings: it is the same camera on the same mount.
 * The goal tag's ID and height come from the field: see docs/LIMELIGHT_APRILTAG.md.
 */
@Configurable
public class AprilTagSettings {

    // ---- Limelight ----

    /** Pipeline slot (0 to 9) holding the AprilTag pipeline in the Limelight web interface. Must not be a slot another pipeline uses. */
    public static int APRILTAG_PIPELINE = 3;

    /** Ignore results older than this many milliseconds. */
    public static long MAX_STALENESS_MS = 100;

    // ---- Which tag ----

    /**
     * ID of the tag on the RED alliance's goal, and on the BLUE alliance's. -1 = aim at whichever tag is biggest in the picture, which
     * is fine for a bench test but will pick the wrong tag on a field with several in view. Set both from the tag pictures.
     */
    public static int GOAL_TAG_ID_RED = -1;
    public static int GOAL_TAG_ID_BLUE = -1;

    // ---- Range ----

    /**
     * true = distance from the 3D pose the Limelight works out for the tag (needs the TAG SIZE set correctly in the pipeline). false =
     * distance from the tag's vertical angle in the picture (needs TAG_CENTRE_HEIGHT_IN below, and the camera height and tilt in PollenSettings).
     * Use both in GoalTagTest and see which one matches a tape measure.
     */
    public static boolean USE_POSE_DISTANCE = true;

    /** Height of the tag's centre above the floor, inches. 0 = not known yet (the vertical-angle distance and the visibility warning stay off). */
    public static double TAG_CENTRE_HEIGHT_IN = 0.0;

    /** Added to the camera's distance to give the distance from where you want to measure (the shooter, the robot centre), inches. */
    public static double RANGE_OFFSET_IN = 0.0;

    // ---- Aiming ----

    /** Turn power per degree of error, and its limits. */
    public static double K_TURN = 0.018;
    public static double MAX_TURN = 0.6;
    public static double MIN_TURN = 0.08;

    /** Aimed when the tag is within this many degrees of the crosshair. */
    public static double AIM_TOLERANCE_DEG = 1.5;

    /** The crosshair should be this many degrees to the RIGHT of the tag (use if the shooter is not in line with the camera). */
    public static double AIM_OFFSET_DEG = 0.0;

    /** true if turning goes the wrong way. Check in GoalTagTest first ("aim would turn ..."). */
    public static boolean INVERT_TX = false;

    /** Autonomous: give up aiming after this long, seconds. */
    public static double AUTO_AIM_TIMEOUT_SEC = 1.5;

    /** Autonomous: if no goal tag is in view this long after the aim starts, skip the aim, seconds. */
    public static double AUTO_NO_TAG_SKIP_SEC = 0.4;

    /** The goal tag ID for the alliance set by LeaveAndPark.ROTATE_180 (false = red, true = blue; the switch every autonomous uses). */
    public static int goalTagId() {
        return LeaveAndPark.ROTATE_180 ? GOAL_TAG_ID_BLUE : GOAL_TAG_ID_RED;
    }

    public static TagMath.AimGains aimGains() {
        TagMath.AimGains g = new TagMath.AimGains();
        g.kTurn = K_TURN;
        g.maxTurn = MAX_TURN;
        g.minTurn = MIN_TURN;
        g.toleranceDeg = AIM_TOLERANCE_DEG;
        g.offsetDeg = AIM_OFFSET_DEG;
        g.invert = INVERT_TX;
        return g;
    }
}
