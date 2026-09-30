// @ftc-toolchain generated: hand-authored — robot power/health dashboard, wired into CompTeleOp and CompAuto
package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Live power/health dashboard: battery voltage + estimated charge, per-motor current
 * draw, and simple anomaly detection (stalls, brownouts, noisy/intermittent connections),
 * surfaced to the Driver Station, the Panels dashboard (http://192.168.43.1:8001 on robot
 * wifi), and a CSV event log for post-match review.
 *
 * Auto-discovers every DcMotorEx and VoltageSensor in the hardware map, so it keeps
 * working unchanged as motors are added/renamed. Call update() once per loop.
 *
 * All thresholds below are dashboard-tunable (@Configurable) starting guesses, not
 * measured values — retune them once you've watched real match data.
 */
@Configurable
public class RobotHealthMonitor {

    // --- Battery voltage -> charge % curve (REV 12V battery, rest voltage). ---
    // This is an approximation: voltage sags under load, so treat the % as a rough
    // trend indicator, not a precise fuel gauge. Retune these two arrays if you have
    // better rest-voltage data for your specific packs.
    public static double[] BATTERY_CURVE_VOLTAGE = {13.0, 12.7, 12.4, 12.1, 11.8, 11.5, 11.2, 10.9, 10.6, 10.3, 10.0};
    public static double[] BATTERY_CURVE_PERCENT = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0};

    public static double LOW_BATTERY_PERCENT = 30;
    public static double CRITICAL_BATTERY_PERCENT = 15;
    public static double BROWNOUT_VOLTAGE = 9.5;

    // --- Per-motor stall detection: current stays high while velocity stays ~0. ---
    public static double STALL_CURRENT_AMPS = 5.0;
    public static double STALL_VELOCITY_TICKS_PER_SEC = 5.0;
    public static double STALL_TIME_SEC = 0.75;

    // --- Noisy/intermittent connection detection: current swings wildly at steady commanded power. ---
    public static double NOISY_CURRENT_STDDEV_AMPS = 2.5;
    private static final int WINDOW_SIZE = 25; // rolling sample window per motor

    private final Telemetry dsTelemetry;
    private final TelemetryManager panelsTelemetry;
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();
    private final List<MonitoredMotor> motors = new ArrayList<>();
    private final Set<String> activeWarnings = new LinkedHashSet<>();
    private final Set<String> previousWarnings = new LinkedHashSet<>();

    private final File logFile;
    private final String opModeName;

    private double batteryVoltage = Double.POSITIVE_INFINITY;
    private double batteryPercent = 100;

    public RobotHealthMonitor(HardwareMap hardwareMap, Telemetry dsTelemetry, String opModeName) {
        this.dsTelemetry = dsTelemetry;
        this.panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        this.opModeName = opModeName;

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensors.add(sensor);
        }
        for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
            motors.add(new MonitoredMotor(motor));
        }

        File logDir = new File(AppUtil.ROOT_FOLDER, "stats");
        logFile = new File(logDir, "health_events.csv");
    }

    /** Call once per loop. Use update(Gamepad) instead if you want rumble alerts on new warnings. */
    public void update() {
        update(null);
    }

    /** Call once per loop. Rumbles gamepadToNotify briefly the moment a new warning first appears. */
    public void update(Gamepad gamepadToNotify) {
        sampleBattery();
        activeWarnings.clear();
        checkBatteryWarnings();
        for (MonitoredMotor motor : motors) {
            motor.sample();
            motor.checkWarnings(activeWarnings);
        }

        boolean hasNewWarning = false;
        for (String warning : activeWarnings) {
            if (!previousWarnings.contains(warning)) {
                hasNewWarning = true;
                logEvent(warning);
            }
        }
        previousWarnings.clear();
        previousWarnings.addAll(activeWarnings);

        if (hasNewWarning && gamepadToNotify != null) {
            gamepadToNotify.rumble(300);
        }

        publishTelemetry();
    }

    private void sampleBattery() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double v = sensor.getVoltage();
            if (v > 0) minVoltage = Math.min(minVoltage, v);
        }
        batteryVoltage = minVoltage;
        batteryPercent = voltageToPercent(minVoltage);
    }

    private static double voltageToPercent(double voltage) {
        double[] v = BATTERY_CURVE_VOLTAGE;
        double[] p = BATTERY_CURVE_PERCENT;
        if (voltage >= v[0]) return p[0];
        if (voltage <= v[v.length - 1]) return p[p.length - 1];
        for (int i = 0; i < v.length - 1; i++) {
            if (voltage <= v[i] && voltage >= v[i + 1]) {
                double t = (voltage - v[i]) / (v[i + 1] - v[i]);
                return p[i] + t * (p[i + 1] - p[i]);
            }
        }
        return 0;
    }

    private void checkBatteryWarnings() {
        if (batteryVoltage <= BROWNOUT_VOLTAGE) {
            activeWarnings.add("BROWNOUT: battery at " + String.format(Locale.US, "%.1fV", batteryVoltage)
                    + " -- expect voltage-starved motors/servos");
        } else if (batteryPercent <= CRITICAL_BATTERY_PERCENT) {
            activeWarnings.add("BATTERY CRITICAL: ~" + (int) batteryPercent + "% remaining -- swap battery now");
        } else if (batteryPercent <= LOW_BATTERY_PERCENT) {
            activeWarnings.add("Battery low: ~" + (int) batteryPercent + "% remaining");
        }
    }

    private void logEvent(String warning) {
        try {
            File logDir = logFile.getParentFile();
            if (!logDir.exists() && !logDir.mkdirs()) return;
            boolean writeHeader = !logFile.exists();
            try (FileWriter fw = new FileWriter(logFile, true)) {
                if (writeHeader) fw.write("timestamp,opMode,event\n");
                fw.write(String.format(Locale.US, "%s,%s,\"%s\"\n",
                        new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()),
                        opModeName, warning.replace("\"", "'")));
            }
        } catch (IOException ignored) {
            // Best-effort logging; don't let a full SD card or IO hiccup take down the OpMode.
        }
    }

    private void publishTelemetry() {
        dsTelemetry.addData("Battery", "%.1fV (~%.0f%%)", batteryVoltage, batteryPercent);
        panelsTelemetry.debug("battery/voltage", batteryVoltage);
        panelsTelemetry.debug("battery/percent", batteryPercent);

        for (MonitoredMotor motor : motors) {
            panelsTelemetry.debug("motors/" + motor.name + "/current", motor.lastCurrent);
            panelsTelemetry.debug("motors/" + motor.name + "/velocity", motor.lastVelocity);
        }

        if (activeWarnings.isEmpty()) {
            dsTelemetry.addData("Health", "OK");
        } else {
            for (String warning : activeWarnings) {
                dsTelemetry.addData("WARNING", warning);
            }
        }
        panelsTelemetry.debug("health/warningCount", activeWarnings.size());
        panelsTelemetry.update();
    }

    public double getBatteryVoltage() { return batteryVoltage; }
    public double getBatteryPercent() { return batteryPercent; }
    public boolean isBrownout() { return batteryVoltage <= BROWNOUT_VOLTAGE; }
    public Set<String> getActiveWarnings() { return activeWarnings; }

    /** Per-motor current/velocity history used for stall + noisy-connection detection. */
    private static class MonitoredMotor {
        final DcMotorEx motor;
        final String name;
        final double[] currentWindow = new double[WINDOW_SIZE];
        int windowIndex = 0;
        int windowCount = 0;
        double lastCurrent;
        double lastVelocity;
        double stallStartTimeSec = -1;

        MonitoredMotor(DcMotorEx motor) {
            this.motor = motor;
            this.name = motor.getDeviceName();
        }

        void sample() {
            lastCurrent = motor.getCurrent(CurrentUnit.AMPS);
            lastVelocity = motor.getVelocity();
            currentWindow[windowIndex] = lastCurrent;
            windowIndex = (windowIndex + 1) % WINDOW_SIZE;
            windowCount = Math.min(windowCount + 1, WINDOW_SIZE);
        }

        void checkWarnings(Set<String> warnings) {
            double power = motor.getPower();
            boolean commanded = Math.abs(power) > 0.05;
            boolean highCurrent = lastCurrent >= STALL_CURRENT_AMPS;
            boolean stalled = Math.abs(lastVelocity) <= STALL_VELOCITY_TICKS_PER_SEC;

            if (commanded && highCurrent && stalled) {
                if (stallStartTimeSec < 0) stallStartTimeSec = System.nanoTime() / 1e9;
                double elapsed = System.nanoTime() / 1e9 - stallStartTimeSec;
                if (elapsed >= STALL_TIME_SEC) {
                    warnings.add("STALL: " + name + " drawing " + String.format(Locale.US, "%.1fA", lastCurrent)
                            + " with ~0 velocity -- check for a mechanical bind or disconnected encoder");
                }
            } else {
                stallStartTimeSec = -1;
            }

            if (windowCount >= WINDOW_SIZE) {
                double stddev = stddev(currentWindow);
                if (stddev >= NOISY_CURRENT_STDDEV_AMPS) {
                    warnings.add("NOISY CURRENT: " + name + " swinging by " + String.format(Locale.US, "%.1fA", stddev)
                            + " -- check for a loose connector or failing motor");
                }
            }
        }

        private static double stddev(double[] values) {
            double mean = 0;
            for (double v : values) mean += v;
            mean /= values.length;
            double variance = 0;
            for (double v : values) variance += (v - mean) * (v - mean);
            variance /= values.length;
            return Math.sqrt(variance);
        }
    }
}
