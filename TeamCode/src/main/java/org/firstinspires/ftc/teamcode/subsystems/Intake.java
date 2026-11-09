// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Single-motor intake with forward/reverse control. Works the same from any OpMode, autonomous or teleop.
 */
@Configurable
public class Intake {

    private DcMotorEx intake;

    // Hardware configuration names (must match the Driver Station config)
    public static final String INTAKE_NAME = "intake";

    // --- Tunable constants (live-editable via the dashboard) ---
    public static double INTAKE_POWER = 0.6; // default intake power

    /** true if "in" spins the wrong way: flip it here instead of swapping wires (read when the Intake is created). */
    public static boolean INTAKE_REVERSED = false;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Run the intake inward at the default power. */
    public void intakeIn() {
        intake.setPower(INTAKE_POWER);
    }

    /** Run the intake outward (reject/reverse) at the default power. */
    public void intakeOut() {
        intake.setPower(-INTAKE_POWER);
    }

    /** Cut power to all actuators. Safe to call any time. */
    public void stop() {
        intake.setPower(0);
    }
}
