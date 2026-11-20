package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The intake: one motor held at INTAKE_RPM (900) on velocity control (the motor controller holds the speed), in or out.
 * Works the same from any OpMode, autonomous or teleop: create it with the hardware map, call intakeIn() / intakeOut() / stop().
 */
@Configurable
public class Intake {

    // Hardware configuration name (must match the Driver Hub configuration, case-sensitive)
    public static final String MOTOR_NAME = "intake";

    // --- Tunable constants (live-editable in Panels; the two REVERSED / TICKS ones are read when the Intake is created) ---
    public static double INTAKE_RPM = 900;

    /** true if "in" spins the wrong way: flip it here instead of swapping wires. */
    public static boolean REVERSED = false;

    /** Encoder ticks per revolution of the motor shaft. 0 = take it from the motor type chosen in the Driver Hub configuration (best). */
    public static double TICKS_PER_REV = 0;

    private final VelocityMotor motor;

    public Intake(HardwareMap hardwareMap) {
        motor = new VelocityMotor(hardwareMap, MOTOR_NAME, REVERSED, TICKS_PER_REV, false);
    }

    /** Run inward at INTAKE_RPM. */
    public void intakeIn() {
        motor.setRpm(INTAKE_RPM);
    }

    /** Run outward (reject) at INTAKE_RPM. */
    public void intakeOut() {
        motor.setRpm(-INTAKE_RPM);
    }

    /** Stop. Safe to call any time. */
    public void stop() {
        motor.stop();
    }

    /** Measured intake speed, RPM (negative = running outward). */
    public double getRpm() {
        return motor.getRpm();
    }

    /** null if all is well, else what is wrong with the motor set-up (for the screen). */
    public String problem() {
        return motor.problem(INTAKE_RPM);
    }
}
