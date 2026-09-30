// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * Limelight 3A vision subsystem for AprilTag localization / target tracking (pipeline TBD once season goal known)
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
