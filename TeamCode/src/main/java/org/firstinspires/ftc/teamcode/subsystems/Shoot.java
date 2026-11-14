package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The shooter: one flywheel motor held at SHOOT_RPM (3000), on velocity control (the motor controller holds the speed).
 * Works the same from any OpMode, autonomous or teleop: create it with the hardware map, call spin() / stop().
 */
@Configurable
public class Shoot {

    // Hardware configuration name (must match the Driver Hub configuration, case-sensitive)
    public static final String MOTOR_NAME = "shoot";

    // --- Tunable constants (live-editable in Panels; the two REVERSED / TICKS ones are read when the Shoot is created) ---
    public static double SHOOT_RPM = 3000;

    /** The flywheel counts as up to speed when it is within this many RPM of SHOOT_RPM. */
    public static double AT_SPEED_TOLERANCE_RPM = 100;

    /** true if the wheel throws the wrong way: flip it here instead of swapping wires. */
    public static boolean REVERSED = false;

    /** Encoder ticks per revolution of the motor shaft. 0 = take it from the motor type chosen in the Driver Hub configuration (best). */
    public static double TICKS_PER_REV = 0;

    private final VelocityMotor motor;
    private double targetRpm;

    public Shoot(HardwareMap hardwareMap) {
        motor = new VelocityMotor(hardwareMap, MOTOR_NAME, REVERSED, TICKS_PER_REV, true);
    }

    /** Spin the flywheel up to SHOOT_RPM. */
    public void spin() {
        targetRpm = SHOOT_RPM;
        motor.setRpm(targetRpm);
    }

    /** Let the flywheel coast down. Safe to call any time. */
    public void stop() {
        targetRpm = 0;
        motor.stop();
    }

    /** Measured flywheel speed, RPM. */
    public double getRpm() {
        return motor.getRpm();
    }

    /** True while spinning and within AT_SPEED_TOLERANCE_RPM of the target: ready to shoot. */
    public boolean isAtSpeed() {
        return targetRpm != 0 && Math.abs(motor.getRpm() - targetRpm) <= AT_SPEED_TOLERANCE_RPM;
    }

    /** null if all is well, else what is wrong with the motor set-up (for the screen). */
    public String problem() {
        return motor.problem(SHOOT_RPM);
    }
}
