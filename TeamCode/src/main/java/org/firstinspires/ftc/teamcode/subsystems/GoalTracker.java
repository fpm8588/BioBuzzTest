package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AprilTagSettings;
import org.firstinspires.ftc.teamcode.PollenSettings;
import org.firstinspires.ftc.teamcode.util.TagMath;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the alliance's scoring strip of AprilTags (four tags in a row) with the Limelight and says how far to turn to face the CENTRE
 * of it and how far away it is. Works the same from any OpMode, autonomous or teleop:
 *
 *   GoalTracker goal = GoalTracker.create(hardwareMap);   // once, in init; never throws; the alliance is LeaveAndPark.ROTATE_180
 *   GoalTracker red = GoalTracker.create(hardwareMap, AprilTagSettings.RED_STRIP_FIRST_ID);  // or name the strip
 *   goal.update();                                         // once per loop
 *   if (goal.hasGoal()) { turn = goal.getAimTurn(); range = goal.getDistanceIn(); }
 *
 * If there is no Limelight in the configuration, create() still returns a tracker: available() is false and hasGoal() stays false,
 * so an OpMode keeps working without the camera. Settings: AprilTagSettings (tag ID, aiming gains, range method) and PollenSettings
 * (camera height and tilt).
 */
public class GoalTracker {

    private final Limelight limelight;       // null when there is no Limelight
    private final boolean pipelineOk;
    private List<TagMath.Tag> tags = new ArrayList<>();
    private final int fixedFirstId;          // -1 = follow LeaveAndPark.ROTATE_180
    private TagMath.Strip goal;
    private long ageMs = Long.MAX_VALUE;
    private String rangeSource = "none";

    private GoalTracker(Limelight limelight, boolean pipelineOk, int fixedFirstId) {
        this.limelight = limelight;
        this.pipelineOk = pipelineOk;
        this.fixedFirstId = fixedFirstId;
    }

    /** Connects to the Limelight and selects the AprilTag pipeline (waits up to 3 s for it). The alliance's strip follows LeaveAndPark.ROTATE_180. Never throws. */
    public static GoalTracker create(HardwareMap hardwareMap) {
        return create(hardwareMap, -1);
    }

    /**
     * Same, for a named strip: pass AprilTagSettings.RED_STRIP_FIRST_ID or BLUE_STRIP_FIRST_ID (the first tag ID of the strip), whatever
     * the autonomous alliance switch says. Never throws.
     */
    public static GoalTracker create(HardwareMap hardwareMap, int firstTagId) {
        try {
            Limelight ll = new Limelight(hardwareMap);
            return new GoalTracker(ll, ll.selectPipeline(AprilTagSettings.APRILTAG_PIPELINE, 3000), firstTagId);
        } catch (RuntimeException e) {
            return new GoalTracker(null, false, firstTagId);   // no "limelight" in the configuration
        }
    }

    /** First tag ID of the strip being tracked. */
    public int getFirstTagId() {
        return fixedFirstId >= 0 ? fixedFirstId : AprilTagSettings.firstTagId();
    }

    /** true if a Limelight was found in the configuration. */
    public boolean available() {
        return limelight != null;
    }

    /** true if the Limelight confirmed it is running the AprilTag pipeline. */
    public boolean pipelineConfirmed() {
        return pipelineOk;
    }

    /** Reads the latest frame. Call once per loop. */
    public void update() {
        if (limelight == null) return;
        limelight.update();
        ageMs = limelight.getStalenessMs();
        tags = limelight.getTags(AprilTagSettings.MAX_STALENESS_MS);
        goal = TagMath.strip(tags, getFirstTagId(), AprilTagSettings.TAG_SIZE_IN, PollenSettings.CAMERA_PITCH_DEG);
    }

    /** Is any tag of the alliance's strip in view, in a fresh frame? */
    public boolean hasGoal() {
        return goal != null;
    }

    /** ID of the biggest tag of the strip in view, -1 if none. */
    public int getId() {
        return goal == null ? -1 : goal.bestId;
    }

    /** How many tags of the strip are in view. */
    public int getTagsUsed() {
        return goal == null ? 0 : goal.tagsUsed;
    }

    /** true if the aim point is the strip's centre; false if it is only the middle of the tags in view (TAG_SIZE_IN not set, or no pose). */
    public boolean isCentred() {
        return goal != null && goal.centred;
    }

    /** Degrees the strip's centre is to the right of the crosshair (0 if not in view). */
    public double getTxDeg() {
        return goal == null ? 0 : goal.txDeg;
    }

    /** Degrees the strip is above the crosshair (0 if not in view). */
    public double getTyDeg() {
        return goal == null ? 0 : goal.tyDeg;
    }

    /** All tags in the latest frame, for telemetry. */
    public List<TagMath.Tag> getTagsSeen() {
        return tags;
    }

    public long getAgeMs() {
        return ageMs;
    }

    /** Is the robot facing the strip's centre, within the tolerance? false if no tag of the strip is in view. */
    public boolean isAimed() {
        return goal != null && TagMath.aimed(goal.txDeg, AprilTagSettings.aimGains());
    }

    /** Degrees still to turn (positive = the tag is to the right); 0 if not in view. */
    public double getAimErrorDeg() {
        return goal == null ? 0 : TagMath.aimErrorDeg(goal.txDeg, AprilTagSettings.aimGains());
    }

    /**
     * Turn command to bring the tag to the crosshair, in Pedro's convention (positive = counter-clockwise), for
     * Follower.setTeleOpDrive's turn argument. 0 when the tag is not in view or the robot is aimed.
     */
    public double getAimTurn() {
        return goal == null ? 0 : TagMath.aimTurn(goal.txDeg, AprilTagSettings.aimGains());
    }

    /** Distance along the floor from the camera to the strip's centre, plus RANGE_OFFSET_IN, inches; NaN if there is no tag or no way to tell. */
    public double getDistanceIn() {
        double d = Double.NaN;
        rangeSource = "none";
        if (goal != null) {
            if (AprilTagSettings.USE_POSE_DISTANCE && !Double.isNaN(goal.poseDistanceIn)) {
                d = goal.poseDistanceIn;
                rangeSource = "3D pose";
            } else if (AprilTagSettings.TAG_CENTRE_HEIGHT_IN > 0) {
                d = TagMath.tyDistanceIn(goal.tyDeg, PollenSettings.CAMERA_HEIGHT_IN, PollenSettings.CAMERA_PITCH_DEG,
                        AprilTagSettings.TAG_CENTRE_HEIGHT_IN);
                rangeSource = "vertical angle";
            }
        }
        return Double.isNaN(d) ? d : d + AprilTagSettings.RANGE_OFFSET_IN;
    }

    /** Which method gave the last getDistanceIn(): "3D pose", "vertical angle" or "none". */
    public String getRangeSource() {
        getDistanceIn();   // sets rangeSource
        return rangeSource;
    }

    /** One line for telemetry. */
    public String status() {
        if (limelight == null) return "no Limelight in the configuration";
        if (!pipelineOk) return "AprilTag pipeline " + AprilTagSettings.APRILTAG_PIPELINE + " NOT confirmed";
        int first = getFirstTagId();
        if (goal == null) return tags.isEmpty() ? "no tag in view" : tags.size() + " tag(s) in view, none in the strip (IDs " + first + " to " + (first + TagMath.STRIP_OFFSETS.length - 1) + ")";
        double d = getDistanceIn();
        return String.format("strip %d-%d: %d tag(s)%s, tx %.1f deg, %s in (%s)", first, first + TagMath.STRIP_OFFSETS.length - 1, goal.tagsUsed,
                goal.centred ? "" : " (NOT centred: set TAG_SIZE_IN)", goal.txDeg,
                Double.isNaN(d) ? "range ?" : String.format("%.0f", d), rangeSource);
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
