package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.PollenMath;

/*
 * Pollen vision check. Does NOT drive: no motor is touched, so it is safe on the bench and on the field.
 *
 * Use it to (1) see that the Limelight is connected and running the pollen pipeline, (2) see that it finds a pollen and
 * nothing else, (3) check which way tx goes, and (4) work out the camera pitch. Steps are in docs/LIMELIGHT_POLLEN.md.
 *
 * Needs only the Limelight in the robot configuration, named "limelight". Works with the drive-only configuration too.
 * Camera numbers come from PollenSettings.
 */
@Configurable
@TeleOp(name = "Pollen Vision Test", group = "Test")
public class PollenVisionTest extends LinearOpMode {

    /**
     * Calibration: put ONE pollen on the floor this far from the camera lens, measured along the floor to the point under
     * the lens, in inches, then read "suggested CAMERA_PITCH_DEG" below. 0 = off.
     */
    public static double KNOWN_DISTANCE_IN = 0.0;

    @Override
    public void runOpMode() {
        Limelight limelight = new Limelight(hardwareMap);
        boolean pipelineOk = limelight.selectPipeline(PollenSettings.POLLEN_PIPELINE, 3000);

        telemetry.addData("Limelight", limelight.statusLine());
        telemetry.addData("pipeline " + PollenSettings.POLLEN_PIPELINE, pipelineOk ? "selected" : "NOT confirmed (check the cable and the config name)");
        double nearest = PollenMath.nearestVisibleDistanceIn(PollenSettings.CAMERA_HEIGHT_IN, PollenSettings.CAMERA_PITCH_DEG,
                PollenSettings.CAMERA_VERTICAL_FOV_DEG, PollenSettings.POLLEN_CENTRE_HEIGHT_IN);
        telemetry.addData("nearest pollen the camera can see (in)", Double.isNaN(nearest) ? "unknown" : String.format("%.1f", nearest));
        if (!Double.isNaN(nearest) && PollenSettings.STOP_DISTANCE_IN < nearest + 1.0) {
            telemetry.addLine("WARNING: STOP_DISTANCE_IN is at or below that, so the pollen will leave the picture before the robot stops."
                    + " Raise STOP_DISTANCE_IN or tilt the camera down more.");
        }
        telemetry.addLine("Press START, then hold a yellow pollen in front of the camera.");
        telemetry.update();

        waitForStart();

        boolean haveLast = false;
        double lastTx = 0;
        try {
            while (opModeIsActive()) {
                limelight.update();
                Limelight.Sighting s = limelight.getPollen(PollenSettings.MIN_AREA, PollenSettings.MAX_STALENESS_MS, haveLast, lastTx, 5.0);

                telemetry.addData("Limelight", limelight.statusLine());
                telemetry.addData("frame age (ms)", limelight.getStalenessMs() == Long.MAX_VALUE ? "no result" : String.valueOf(limelight.getStalenessMs()));

                if (s == null) {
                    haveLast = false;
                    telemetry.addData("pollen", "NONE (nothing yellow big enough in view, a stale frame, or the pipeline is wrong)");
                } else {
                    haveLast = true;
                    lastTx = s.txDeg;
                    double dist = PollenSettings.distanceIn(s.tyDeg);
                    PollenMath.Command c = PollenMath.command(s.txDeg, dist, PollenSettings.gains());

                    telemetry.addData("pollen", "FOUND (%d blob%s in view)", s.blobsSeen, s.blobsSeen == 1 ? "" : "s");
                    telemetry.addData("tx (deg, + = right)", "%.1f", s.txDeg);
                    telemetry.addData("ty (deg, + = up)", "%.1f", s.tyDeg);
                    telemetry.addData("area (% of image)", "%.3f", s.area * 100);
                    telemetry.addData("distance from camera (in)", Double.isNaN(dist) ? "unknown (check pitch and height)" : String.format("%.1f", dist));
                    telemetry.addData("chase would", "%s, %s%s",
                            c.turn > 0 ? "turn LEFT" : c.turn < 0 ? "turn RIGHT" : "not turn",
                            c.forward > 0 ? "drive forward" : "not drive forward",
                            c.arrived ? ", ARRIVED (inside the stop distance)" : "");

                    if (KNOWN_DISTANCE_IN > 0) {
                        double pitch = PollenMath.pitchFromKnownDistance(PollenSettings.CAMERA_HEIGHT_IN, KNOWN_DISTANCE_IN,
                                s.tyDeg, PollenSettings.POLLEN_CENTRE_HEIGHT_IN);
                        telemetry.addData("suggested CAMERA_PITCH_DEG", "%.1f (now %.1f)", pitch, PollenSettings.CAMERA_PITCH_DEG);
                    }
                }
                telemetry.update();
            }
        } finally {
            limelight.stop();
        }
    }
}
