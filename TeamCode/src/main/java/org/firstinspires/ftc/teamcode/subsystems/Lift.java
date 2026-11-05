// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Single-servo lift with three discrete positions
 */
@Configurable
public class Lift {

    private Servo lift;

    // Hardware configuration names (must match the Driver Station config)
    public static final String LIFT_NAME = "lift";

    // --- Tunable constants (live-editable via the dashboard) ---
    public static double POSITION_UP = 1.0;
    public static double POSITION_MID = 0.5;
    public static double POSITION_DOWN = 0.0;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(Servo.class, LIFT_NAME);
    }

    /** Move the lift servo to its top position. */
    public void goUp() {
        lift.setPosition(POSITION_UP);
    }

    /** Move the lift servo to its middle position. */
    public void goMid() {
        lift.setPosition(POSITION_MID);
    }

    /** Move the lift servo to its bottom position. */
    public void goDown() {
        lift.setPosition(POSITION_DOWN);
    }

    /** Cut power to all actuators. Safe to call any time. */
    public void stop() {
        // No powered actuators to stop.
    }
}
