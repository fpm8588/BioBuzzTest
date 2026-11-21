package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.ObstacleMath;
import org.firstinspires.ftc.teamcode.util.PollenMath;

import java.util.ArrayList;
import java.util.List;

/*
 * Obstacle vision check. Does NOT drive: no motor is touched, so it is safe on the bench and on the field.
 *
 * Use it to calibrate the camera for AVOIDANDPARK: see that the colour pipeline finds a robot sign and nothing else, check
 * which way tx goes, find DISTANCE_K, and see what the robot would think is in its way. Steps: docs/LIMELIGHT_OBSTACLES.md.
 * Needs only the Limelight in the robot configuration, named "limelight". All numbers come from ObstacleSettings.
 *
 * Positions are shown relative to the robot (forward and left of its centre), because there is no localizer here.
 */
@Configurable
@TeleOp(name = "Obstacle Vision Test", group = "Test")
public class ObstacleVisionTest extends LinearOpMode {

    /** Pipeline slot to test. -1 = what AVOIDANDPARK uses (ObstacleSettings.PIPELINE_OVERRIDE, else the opposing colour's slot). */
    public static int PIPELINE_OVERRIDE = -1;

    /**
     * Calibration: put the thing to detect (a robot's sign or bumper, seen the way it will be seen) this far in front of the
     * camera lens, measured along the floor, in inches, then read "suggested DISTANCE_K". 0 = off.
     */
    public static double KNOWN_DISTANCE_IN = 0.0;

    @Override
    public void runOpMode() {
        Limelight limelight = new Limelight(hardwareMap);
        int slot = PIPELINE_OVERRIDE >= 0 ? PIPELINE_OVERRIDE : ObstacleSettings.opponentPipeline();
        boolean ok = limelight.selectPipeline(slot, 3000);

        telemetry.addData("Limelight", limelight.statusLine());
        telemetry.addData("pipeline " + slot, ok ? "selected" : "NOT confirmed (check the cable and the config name)");
        double maxRange = ObstacleMath.distanceFromArea(ObstacleSettings.MIN_AREA, ObstacleSettings.DISTANCE_K);
        telemetry.addData("farthest it can see with MIN_AREA (in)", Double.isNaN(maxRange) ? "unknown" : String.format("%.0f", maxRange));
        double far = ObstacleMath.farthestVisibleDistanceIn(PollenSettings.CAMERA_HEIGHT_IN, PollenSettings.CAMERA_PITCH_DEG,
                PollenSettings.CAMERA_VERTICAL_FOV_DEG, ObstacleSettings.TYPICAL_SIGN_HEIGHT_IN);
        double need = ObstacleSettings.TRIGGER_RANGE_IN + ObstacleSettings.OBSTACLE_RADIUS_IN + ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN;
        telemetry.addData("farthest a sign at that height stays in the picture (in)", Double.isInfinite(far) ? "no limit" : Double.isNaN(far) ? "unknown (camera below the sign?)" : String.format("%.0f", far));
        if (!Double.isNaN(far) && far < need) {
            telemetry.addLine("WARNING: the camera is tilted down so far that a robot sign leaves the top of the picture beyond " + Math.round(far)
                    + " in, but the robot needs to see " + Math.round(need) + " in ahead. Tilt the camera up, raise it, or lower TRIGGER_RANGE_IN.");
        }
        telemetry.addLine("Press START, then put a robot sign of the colour in front of the camera.");
        telemetry.update();

        waitForStart();

        double clearance = ObstacleSettings.OBSTACLE_RADIUS_IN + ObstacleSettings.ROBOT_CLEARANCE_RADIUS_IN + ObstacleSettings.CLEARANCE_MARGIN_IN;
        try {
            while (opModeIsActive()) {
                limelight.update();
                List<PollenMath.Blob> blobs = limelight.getBlobs(ObstacleSettings.MAX_STALENESS_MS);

                telemetry.addData("Limelight", limelight.statusLine());
                telemetry.addData("frame age (ms)", limelight.getStalenessMs() == Long.MAX_VALUE ? "no result" : String.valueOf(limelight.getStalenessMs()));
                telemetry.addData("blobs in view", blobs.size());

                PollenMath.Blob biggest = null;
                List<String> shown = new ArrayList<>();
                int n = 0;
                for (PollenMath.Blob b : blobs) {
                    if (biggest == null || b.area > biggest.area) biggest = b;
                    double dist = ObstacleMath.distanceFromArea(b.area, ObstacleSettings.DISTANCE_K);
                    boolean tooSmall = b.area < ObstacleSettings.MIN_AREA;
                    double tx = ObstacleSettings.INVERT_TX ? -b.txDeg : b.txDeg;
                    // Robot frame: forward and left of the centre, camera ahead of the centre by the offset.
                    double bearing = -Math.toRadians(tx); // right of the crosshair = clockwise
                    double fwd = ObstacleSettings.CAMERA_FORWARD_OFFSET_IN + dist * Math.cos(bearing);
                    double left = dist * Math.sin(bearing);
                    boolean inWay = !tooSmall && fwd > 0 && fwd < ObstacleSettings.TRIGGER_RANGE_IN + clearance && Math.abs(left) < clearance;
                    if (n++ < 4) {
                        shown.add(String.format("tx %.1f  ty %.1f  area %.3f%%  ~%.0f in: %.0f fwd, %.0f %s%s%s", b.txDeg, b.tyDeg, b.area * 100, dist,
                                fwd, Math.abs(left), left >= 0 ? "left" : "right", tooSmall ? "  [too small, ignored]" : "", inWay ? "  ** IN THE WAY **" : ""));
                    }
                }
                if (blobs.isEmpty()) {
                    telemetry.addData("patch", "NONE (nothing of the colour big enough, a stale frame, or the wrong pipeline)");
                } else {
                    for (int i = 0; i < shown.size(); i++) telemetry.addData("patch " + (i + 1), shown.get(i));
                }

                if (KNOWN_DISTANCE_IN > 0 && biggest != null) {
                    telemetry.addData("suggested DISTANCE_K", "%.2f (now %.2f)", ObstacleMath.kFromKnown(KNOWN_DISTANCE_IN, biggest.area), ObstacleSettings.DISTANCE_K);
                }
                telemetry.addData("room needed beside an obstacle (in)", "%.0f  (its radius + ours + margin)", clearance);
                telemetry.update();
            }
        } finally {
            limelight.stop();
        }
    }
}
