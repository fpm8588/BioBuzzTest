package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.PollenMath;

/*
 * Every number the pollen vision depends on, in one place, shared by PollenVisionTest and PollenChase.
 * Change them in Panels (Configurables > PollenSettings), or here and rebuild.
 *
 * Camera numbers (height, pitch) are measured on the robot: see docs/LIMELIGHT_POLLEN.md, steps 8 and 9.
 */
@Configurable
public class PollenSettings {

    // ---- Limelight ----

    /** Pipeline slot (0 to 9) holding the yellow pollen colour pipeline in the Limelight web interface. */
    public static int POLLEN_PIPELINE = 0;

    /** Ignore results older than this many milliseconds (the Limelight docs use 100 as an example). */
    public static long MAX_STALENESS_MS = 100;

    /** Ignore blobs smaller than this fraction of the image (0 to 1). 0.0005 is 0.05 percent. Matches the pipeline's area filter. */
    public static double MIN_AREA = 0.0005;

    // ---- Camera mounting, measured on the robot ----

    /** Height of the camera lens above the floor, inches. */
    public static double CAMERA_HEIGHT_IN = 10.0;

    /** How far the camera looks below horizontal, degrees (tilted down = positive). Use the calibration in PollenVisionTest. */
    public static double CAMERA_PITCH_DEG = 25.0;

    /**
     * Vertical field of view of the camera, degrees. Only used to warn when the stop distance is closer than the camera
     * can see. 42 is the Limelight 3A figure as I remember it: check it against the Limelight 3A spec sheet.
     */
    public static double CAMERA_VERTICAL_FOV_DEG = 42.0;

    /** Height of the pollen's centre above the floor, inches (a 2.8 in ball rests with its centre 1.4 in up). */
    public static double POLLEN_CENTRE_HEIGHT_IN = PollenMath.POLLEN_RADIUS_IN;

    // ---- Chase behaviour (PollenChase) ----

    /** Stop when the pollen is this far from the camera along the floor, inches. Set so the intake just reaches it. */
    public static double STOP_DISTANCE_IN = 10.0;

    /** Turn power per degree of tx, and its limits. */
    public static double K_TURN = 0.018;
    public static double MAX_TURN = 0.6;
    public static double MIN_TURN = 0.08;
    public static double AIM_TOLERANCE_DEG = 1.5;

    /** Forward power per inch still to go, and its limits. */
    public static double K_FORWARD = 0.04;
    public static double MAX_FORWARD = 0.9;
    public static double MIN_FORWARD = 0.15;

    /** Only drive forward when the pollen is within this many degrees of the crosshair. */
    public static double ALIGN_WINDOW_DEG = 20.0;

    /** true if turning goes the wrong way (camera mounted so tx has the opposite sign). Check in PollenVisionTest first. */
    public static boolean INVERT_TX = false;

    /** Overall motor power cap for the chase (0 to 1). This caps MAX_FORWARD and MAX_TURN: a low value here is a slow robot. */
    public static double CHASE_MAX_POWER = 0.9;

    /** Camera delay the controller allows for when it brakes, seconds. Raise it if the robot overshoots, lower it if it creeps. */
    public static double LATENCY_LEAD_SEC = 0.15;

    /** Robot speed at full power, inches per second (the Forward Velocity tuner gave 64). */
    public static double FULL_POWER_SPEED_IN_S = 64.0;

    /** The pollen must be seen continuously this long before the robot reacts to it, seconds. */
    public static double ACQUIRE_TIME_SEC = 0.10;

    /** If the pollen disappears for this long while chasing, go back to searching, seconds. */
    public static double LOST_TIMEOUT_SEC = 0.5;

    /**
     * If the pollen disappears when it was last seen within STOP_DISTANCE_IN plus this many inches, it has gone under the
     * bottom of the camera's view because the robot reached it: count that as ARRIVED instead of searching again.
     */
    public static double NEAR_LOST_MARGIN_IN = 6.0;

    /** While searching the robot spins in place at this power. Positive = counter-clockwise (left), negative = clockwise. */
    public static double SEARCH_TURN_POWER = 0.3;

    /** Give up searching after this long, seconds. */
    public static double SEARCH_TIMEOUT_SEC = 6.0;

    /** DRIVEANDGRAB: wait this long after the drive finishes before the chase starts, so the robot has settled, seconds. */
    public static double PAUSE_BEFORE_CHASE_SEC = 0.3;

    /** Give up on the whole run after this long, seconds. Autonomous lasts 30. */
    public static double RUN_TIMEOUT_SEC = 20.0;

    /** Never let the robot get further than this from where it started, inches. */
    public static double MAX_TRAVEL_IN = 70.0;

    // ---- Where the robot starts (PollenChase), in the same field coordinates as Leave and Park. ----
    // The robot spins in place while searching, so it must start on open floor: a square robot turning on the spot sweeps
    // a circle about 1.4 times its half-width, which would go through a wall or a field element it starts next to.
    // Default: 20 in from the bottom wall, facing +y, clear of the HIVE and flowers.
    public static double START_X = 56.0;
    public static double START_Y = 20.0;
    public static double START_HEADING_DEG = 90.0;

    /**
     * true = also stop if the robot would touch a wall, the HIVE or a FLOWER (positions from LeaveAndPark). Set false to
     * test on a bench or an empty floor with no field; the travel limit and run timeout still apply.
     */
    public static boolean ENABLE_FIELD_GUARD = true;

    static PollenMath.Gains gains() {
        PollenMath.Gains g = new PollenMath.Gains();
        g.kTurn = K_TURN;
        g.maxTurn = MAX_TURN;
        g.minTurn = MIN_TURN;
        g.aimToleranceDeg = AIM_TOLERANCE_DEG;
        g.kForward = K_FORWARD;
        g.maxForward = MAX_FORWARD;
        g.minForward = MIN_FORWARD;
        g.stopDistanceIn = STOP_DISTANCE_IN;
        g.alignWindowDeg = ALIGN_WINDOW_DEG;
        g.invertTx = INVERT_TX;
        g.latencyLeadSec = LATENCY_LEAD_SEC;
        g.fullPowerSpeedInPerSec = FULL_POWER_SPEED_IN_S;
        return g;
    }

    /** Floor distance from the camera to a pollen seen at ty degrees, using the camera numbers above. NaN if not on the floor. */
    static double distanceIn(double tyDeg) {
        return PollenMath.floorDistanceIn(CAMERA_HEIGHT_IN, CAMERA_PITCH_DEG, tyDeg, POLLEN_CENTRE_HEIGHT_IN);
    }
}
