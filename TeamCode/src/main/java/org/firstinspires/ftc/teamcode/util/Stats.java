package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Match statistics: tracks cycle counts, shooting accuracy, and drive distance,
 * then surfaces them to the Driver Station, the Panels dashboard, and a CSV log
 * on the Robot Controller for post-match review.
 *
 * One instance per OpMode run. Call update() once per loop, and writeSummaryToFile()
 * from your OpMode's stop() so a row is written even if the match ends abruptly.
 */
public class Stats {

    private static final File LOG_DIR = new File(AppUtil.ROOT_FOLDER, "stats");
    private static final File LOG_FILE = new File(LOG_DIR, "match_stats.csv");

    private final Telemetry dsTelemetry;
    private final TelemetryManager panelsTelemetry;
    private final ElapsedTime matchTimer = new ElapsedTime();

    private int shotsAttempted = 0;
    private int shotsScored = 0;
    private int intakeCycles = 0;
    private double driveDistanceIn = 0;
    private Pose lastPose;

    public Stats(Telemetry dsTelemetry) {
        this.dsTelemetry = dsTelemetry;
        this.panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /** Record one shot attempt. */
    public void recordShot(boolean scored) {
        shotsAttempted++;
        if (scored) shotsScored++;
    }

    /** Record one intake in/out cycle. */
    public void recordIntakeCycle() {
        intakeCycles++;
    }

    /** Feed the current follower pose each loop to accumulate distance traveled. */
    public void trackPose(Pose currentPose) {
        if (lastPose != null) {
            driveDistanceIn += Math.hypot(
                    currentPose.getX() - lastPose.getX(),
                    currentPose.getY() - lastPose.getY());
        }
        lastPose = currentPose;
    }

    public double shootingAccuracy() {
        return shotsAttempted == 0 ? 0.0 : (double) shotsScored / shotsAttempted;
    }

    /** Push current stats to both the Driver Station telemetry and the Panels dashboard. Call once per loop. */
    public void update() {
        dsTelemetry.addData("Match Time (s)", "%.1f", matchTimer.seconds());
        dsTelemetry.addData("Shots", shotsScored + "/" + shotsAttempted);
        dsTelemetry.addData("Accuracy", "%.0f%%", shootingAccuracy() * 100);
        dsTelemetry.addData("Intake Cycles", intakeCycles);
        dsTelemetry.addData("Drive Distance (in)", "%.1f", driveDistanceIn);

        panelsTelemetry.debug("matchTimeSec", matchTimer.seconds());
        panelsTelemetry.debug("shotsScored", shotsScored);
        panelsTelemetry.debug("shotsAttempted", shotsAttempted);
        panelsTelemetry.debug("accuracy", shootingAccuracy());
        panelsTelemetry.debug("intakeCycles", intakeCycles);
        panelsTelemetry.debug("driveDistanceIn", driveDistanceIn);
        panelsTelemetry.update();
    }

    /** Append one summary row to stats/match_stats.csv on the Robot Controller. Call from stop(). */
    public void writeSummaryToFile(String opModeName) {
        try {
            if (!LOG_DIR.exists() && !LOG_DIR.mkdirs()) {
                dsTelemetry.addData("Stats", "Could not create log directory");
                return;
            }
            boolean writeHeader = !LOG_FILE.exists();
            try (FileWriter fw = new FileWriter(LOG_FILE, true)) {
                if (writeHeader) {
                    fw.write("timestamp,opMode,matchTimeSec,shotsScored,shotsAttempted,accuracy,intakeCycles,driveDistanceIn\n");
                }
                fw.write(String.format(Locale.US, "%s,%s,%.1f,%d,%d,%.3f,%d,%.1f\n",
                        new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()),
                        opModeName, matchTimer.seconds(), shotsScored, shotsAttempted,
                        shootingAccuracy(), intakeCycles, driveDistanceIn));
            }
        } catch (IOException e) {
            dsTelemetry.addData("Stats", "Failed to write log: " + e.getMessage());
        }
    }
}
