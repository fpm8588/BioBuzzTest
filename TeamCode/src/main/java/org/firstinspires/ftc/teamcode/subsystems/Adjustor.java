// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Single-servo shot-angle adjustor with three preset positions and free positioning. Works the same from any OpMode,
 * autonomous or teleop.
 */
@Configurable
public class Adjustor {

    private Servo adjustor;
    private double position;
    private final boolean enabled;   // AuxSettings.ENABLED when created: false = no hardware, every command does nothing

    // Hardware configuration names (must match the Driver Station config)
    public static final String ADJUSTOR_NAME = "adjustor";

    // --- Tunable constants (live-editable via the dashboard) ---
    public static double POSITION_HIGH = 1.0;
    public static double POSITION_MID = 0.5;
    public static double POSITION_LOW = 0.0;

    /** The servo never goes outside these, whatever is asked (protects the mechanism). */
    public static double MIN_POSITION = 0.0;
    public static double MAX_POSITION = 1.0;

    public Adjustor(HardwareMap hardwareMap) {
        enabled = AuxSettings.ENABLED;
        position = Double.NaN;
        if (!enabled) return;
        adjustor = hardwareMap.get(Servo.class, ADJUSTOR_NAME);
        position = Double.NaN; // a servo cannot report where it is until it has been told
    }

    /** false when AuxSettings.ENABLED was off at creation: nothing is connected and nothing moves. */
    public boolean isEnabled() {
        return enabled;
    }

    /** Move the adjustor to its high preset. */
    public void goHigh() {
        setPosition(POSITION_HIGH);
    }

    /** Move the adjustor to its middle preset. */
    public void goMid() {
        setPosition(POSITION_MID);
    }

    /** Move the adjustor to its low preset. */
    public void goLow() {
        setPosition(POSITION_LOW);
    }

    /** Move the adjustor to any position (0 to 1, limited to MIN_POSITION..MAX_POSITION). */
    public void setPosition(double target) {
        if (!enabled) return;
        position = Math.max(MIN_POSITION, Math.min(MAX_POSITION, target));
        adjustor.setPosition(position);
    }

    /** Move the adjustor by delta from where it was last told to go (the middle preset if it has not been told yet). */
    public void nudge(double delta) {
        setPosition((Double.isNaN(position) ? POSITION_MID : position) + delta);
    }

    /** Where the adjustor was last told to go (NaN before the first command). A servo gives no feedback. */
    public double getPosition() {
        return position;
    }

    /** Cut power to all actuators. Safe to call any time. */
    public void stop() {
        // No powered actuators to stop.
    }
}
