package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AprilTagSettings;
import org.firstinspires.ftc.teamcode.PollenSettings;
import org.firstinspires.ftc.teamcode.util.TagMath;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the alliance's goal AprilTag with the Limelight and says how far to turn to face it and how far away it is. Works the same
 * from any OpMode, autonomous or teleop:
 *
 *   GoalTracker goal = GoalTracker.create(hardwareMap);   // once, in init; never throws
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
    private TagMath.Tag goal;
    private long ageMs = Long.MAX_VALUE;
    private String rangeSource = "none";

    private GoalTracker(Limelight limelight, boolean pipelineOk) {
        this.limelight = limelight;
        this.pipelineOk = pipelineOk;
    }

    /** Connects to the Limelight and selects the AprilTag pipeline (waits up to 3 s for it). Never throws. */
    public static GoalTracker create(HardwareMap hardwareMap) {
        try {
            Limelight ll = new Limelight(hardwareMap);
            return new GoalTracker(ll, ll.selectPipeline(AprilTagSettings.APRILTAG_PIPELINE, 3000));
        } catch (RuntimeException e) {
            return new GoalTracker(null, false);   // no "limelight" in the configuration
        }
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
        goal = TagMath.pick(tags, AprilTagSettings.goalTagId());
    }

    /** Is the alliance's goal tag in view, in a fresh frame? */
    public boolean hasGoal() {
        return goal != null;
    }

    public int getId() {
        return goal == null ? -1 : goal.id;
    }

    /** Degrees the tag is to the right of the crosshair (0 if not in view). */
    public double getTxDeg() {
        return goal == null ? 0 : goal.txDeg;
    }

    /** Degrees the tag is above the crosshair (0 if not in view). */
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

    /** Is the robot facing the tag, within the tolerance? false if the tag is not in view. */
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

    /** Distance along the floor from the camera to the tag, plus RANGE_OFFSET_IN, inches; NaN if there is no tag or no way to tell. */
    public double getDistanceIn() {
        double d = Double.NaN;
        rangeSource = "none";
        if (goal != null) {
            if (AprilTagSettings.USE_POSE_DISTANCE && goal.hasPose()) {
                d = TagMath.poseDistanceIn(goal.camXIn, goal.camYIn, goal.camZIn, PollenSettings.CAMERA_PITCH_DEG);
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
        return rangeSource;
    }

    /** One line for telemetry. */
    public String status() {
        if (limelight == null) return "no Limelight in the configuration";
        if (!pipelineOk) return "AprilTag pipeline " + AprilTagSettings.APRILTAG_PIPELINE + " NOT confirmed";
        if (goal == null) return tags.isEmpty() ? "no tag in view" : tags.size() + " tag(s) in view, none is the goal (ID " + AprilTagSettings.goalTagId() + ")";
        double d = getDistanceIn();
        return String.format("goal tag %d: tx %.1f deg, %s in (%s)", goal.id, goal.txDeg,
                Double.isNaN(d) ? "range ?" : String.format("%.0f", d), rangeSource);
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
