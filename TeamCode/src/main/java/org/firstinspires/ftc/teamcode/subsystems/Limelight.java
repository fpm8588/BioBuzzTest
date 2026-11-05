// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.PollenMath;

import java.util.ArrayList;
import java.util.List;

/**
 * Limelight 3A vision subsystem for AprilTag localization / target tracking (pipeline TBD once season goal known).
 *
 * Also finds POLLEN with a yellow colour pipeline: see getPollen(). Since SDK 10.3 getLatestResult() is never null, so
 * "no data" shows up as isValid() == false, and results can be old, so getPollen() also checks the staleness.
 */
public class Limelight {

    private final Limelight3A limelight;
    private LLResult latestResult;

    // Hardware configuration names (must match the Driver Station config)
    public static final String LIMELIGHT_NAME = "limelight";

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    /** Same as the plain constructor, then selects the given pipeline slot. Use selectPipeline() to wait for it. */
    public Limelight(HardwareMap hardwareMap, int pipelineIndex) {
        this(hardwareMap);
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Switches to a pipeline and waits until the Limelight reports that it is running it, because results that arrive
     * right after a switch still come from the old pipeline.
     *
     * @return true once the Limelight says the pipeline is active; false on timeout (is the Limelight connected?)
     */
    public boolean selectPipeline(int index, long timeoutMs) {
        limelight.pipelineSwitch(index);
        long end = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < end) {
            LLStatus status = limelight.getStatus();
            if (status != null && status.getPipelineIndex() == index) return true;
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        return false;
    }

    /** One line about the Limelight itself for telemetry: name, temperature, CPU, frame rate, running pipeline. */
    public String statusLine() {
        LLStatus s = limelight.getStatus();
        if (s == null) return "no status (not connected?)";
        return String.format("%s  %.0fC  CPU %.0f%%  %d FPS  pipeline %d (%s)",
                s.getName(), s.getTemp(), s.getCpu(), (int) s.getFps(), s.getPipelineIndex(), s.getPipelineType());
    }

    /** Age of the last result in milliseconds, or a huge number if there is none. */
    public long getStalenessMs() {
        return latestResult == null ? Long.MAX_VALUE : (long) latestResult.getStaleness();
    }

    /** A pollen the camera is looking at: angles from the crosshair, size as a fraction of the image, and frame age. */
    public static final class Sighting {
        public final double txDeg, tyDeg, area;
        public final long ageMs;
        public final int blobsSeen;

        Sighting(double txDeg, double tyDeg, double area, long ageMs, int blobsSeen) {
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
            this.ageMs = ageMs;
            this.blobsSeen = blobsSeen;
        }
    }

    /**
     * The pollen to go for, from the colour pipeline's blobs, or null if there is none worth acting on: no valid
     * result, a result older than maxStalenessMs, or only blobs smaller than minArea. Call update() first.
     *
     * @param haveLast  true if you are already following a pollen; then a blob within stickyDeg of lastTxDeg is kept
     */
    public Sighting getPollen(double minArea, long maxStalenessMs, boolean haveLast, double lastTxDeg, double stickyDeg) {
        if (latestResult == null || !latestResult.isValid()) return null;
        long age = getStalenessMs();
        if (age > maxStalenessMs) return null;

        List<PollenMath.Blob> blobs = new ArrayList<>();
        for (LLResultTypes.ColorResult c : latestResult.getColorResults()) {
            blobs.add(new PollenMath.Blob(c.getTargetXDegrees(), c.getTargetYDegrees(), c.getTargetArea()));
        }
        PollenMath.Blob best = PollenMath.pick(blobs, minArea, haveLast, lastTxDeg, stickyDeg);
        if (best == null) return null;
        return new Sighting(best.txDeg, best.tyDeg, best.area, age, blobs.size());
    }

    /**
     * Every blob the colour pipeline found in the latest frame (tx and ty in degrees, area as a fraction of the image),
     * or an empty list if there is no valid result or it is older than maxStalenessMs. Call update() first.
     */
    public List<PollenMath.Blob> getBlobs(long maxStalenessMs) {
        List<PollenMath.Blob> blobs = new ArrayList<>();
        if (latestResult == null || !latestResult.isValid() || getStalenessMs() > maxStalenessMs) return blobs;
        for (LLResultTypes.ColorResult c : latestResult.getColorResults()) {
            blobs.add(new PollenMath.Blob(c.getTargetXDegrees(), c.getTargetYDegrees(), c.getTargetArea()));
        }
        return blobs;
    }

    /** Pulls the latest frame result from the Limelight. Call once per loop before reading targets. */
    public void update() {
        latestResult = limelight.getLatestResult();
    }

    /** Horizontal offset (degrees) from crosshair to target. 0 if no target. */
    public double getTargetX() {
        return hasTarget() ? latestResult.getTx() : 0.0;
    }

    /** Vertical offset (degrees) from crosshair to target. 0 if no target. */
    public double getTargetY() {
        return hasTarget() ? latestResult.getTy() : 0.0;
    }

    /** Whether the last update() saw a valid target. */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    /** Field-relative robot pose from AprilTag localization, or null if unavailable. */
    public Pose3D getBotPose() {
        return hasTarget() ? latestResult.getBotpose() : null;
    }

    /** Switch the active vision pipeline (e.g. different AprilTag family or color pipeline). */
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    /** Cut power to all actuators. Safe to call any time. */
    public void stop() {
        limelight.stop();
    }
}
