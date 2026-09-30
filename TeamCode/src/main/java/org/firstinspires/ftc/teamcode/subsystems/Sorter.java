// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Dual-servo sorting gate that routes game pieces left/right
 */
@Configurable
public class Sorter {

    private Servo sortOne;
    private Servo sortTwo;

    // Hardware configuration names (must match the Driver Station config)
    public static final String SORT_ONE_NAME = "sortOne";
    public static final String SORT_TWO_NAME = "sortTwo";

    // --- Tunable constants (live-editable via the dashboard) ---
    public static double GATE_OPEN = 1.0;
    public static double GATE_CLOSED = 0.0;

    public Sorter(HardwareMap hardwareMap) {
        sortOne = hardwareMap.get(Servo.class, SORT_ONE_NAME);
        sortTwo = hardwareMap.get(Servo.class, SORT_TWO_NAME);
    }

    /** Open the left gate, close the right. */
    public void sortLeft() {
        sortOne.setPosition(GATE_OPEN);
        sortTwo.setPosition(GATE_CLOSED);
    }

    /** Open the right gate, close the left. */
    public void sortRight() {
        sortTwo.setPosition(GATE_OPEN);
        sortOne.setPosition(GATE_CLOSED);
    }

    /** Close both gates. */
    public void neutral() {
        sortOne.setPosition(GATE_CLOSED);
        sortTwo.setPosition(GATE_CLOSED);
    }

    /** Cut power to all actuators. Safe to call any time. */
    public void stop() {
        // No powered actuators to stop.
    }
}
