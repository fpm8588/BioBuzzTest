package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.TagMath;

/*
 * Every number the goal-tag aiming and ranging depends on, in one place, shared by GoalTagTest, the red and blue TeleOps (CompTeleOpBase) and CompAuto.
 * Change them in Panels (Configurables > AprilTagSettings), or here and rebuild.
 *
 * The camera itself (height above the floor, tilt, lens angle) is the one in PollenSettings: it is the same camera on the same mount.
 * The tag IDs are from the printed strips; the tag size and height come from the field: see docs/LIMELIGHT_APRILTAG.md.
 */
@Configurable
public class AprilTagSettings {

    // ---- Limelight ----

    /** Pipeline slot (0 to 9) holding the AprilTag pipeline in the Limelight web interface. Slot 2 is the team's AprilTag pipeline. It must not be a slot another pipeline uses (ObstacleSettings.PIPELINE_BLUE_ROBOTS is also 2: it is only used if PIPELINE_OVERRIDE is turned off). */
    public static int APRILTAG_PIPELINE = 2;

    /** Ignore results older than this many milliseconds. */
    public static long MAX_STALENESS_MS = 100;

    // ---- Which tags ----

    /**
     * The scoring strips are rows of four tags, IDs running left to right as you face the strip: RED SCORING 30 31 32 33, BLUE SCORING
     * 42 43 44 45 (the audience strips, 34 to 41, are not used for aiming). The robot aims at the CENTRE of its alliance's strip,
     * worked out from whichever of the four are in view.
     */
    public static int RED_STRIP_FIRST_ID = 30;
    public static int BLUE_STRIP_FIRST_ID = 42;

    /**
     * Size of the black square of a tag, edge to edge, in INCHES (the same number as the tag size in the Limelight pipeline, in mm, divided
     * by 25.4). Needed to find the strip's centre when only some of its tags are in view. 0 = not set: the robot then aims at the middle of
     * the tags it can see, which is off-centre by up to two tag widths when only part of the strip is in view.
     */
    public static double TAG_SIZE_IN = 0.0;

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

    /** First tag ID of the alliance's strip for the alliance set by LeaveAndPark.ROTATE_180 (false = red, true = blue; the switch every autonomous uses). */
    public static int firstTagId() {
        return LeaveAndPark.ROTATE_180 ? BLUE_STRIP_FIRST_ID : RED_STRIP_FIRST_ID;
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
