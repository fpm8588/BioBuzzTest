// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Dual-motor intake with forward/reverse control
 */
@Configurable
public class Intake {

    private DcMotorEx inOne;
    private DcMotorEx inTwo;

    // Hardware configuration names (must match the Driver Station config)
    public static final String IN_ONE_NAME = "inOne";
    public static final String IN_TWO_NAME = "inTwo";

    // --- Tunable constants (live-editable via the dashboard) ---
    public static double INTAKE_POWER = 0.6; // default intake power

    public Intake(HardwareMap hardwareMap) {
        inOne = hardwareMap.get(DcMotorEx.class, IN_ONE_NAME);
        inOne.setDirection(DcMotorSimple.Direction.FORWARD);
        inOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inTwo = hardwareMap.get(DcMotorEx.class, IN_TWO_NAME);
        inTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        inTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Run both intake motors inward at the default power. */
    public void intakeIn() {
        inOne.setPower(INTAKE_POWER);
        inTwo.setPower(INTAKE_POWER);
    }

    /** Run both intake motors outward (reject/reverse) at the default power. */
    public void intakeOut() {
        inOne.setPower(-INTAKE_POWER);
        inTwo.setPower(-INTAKE_POWER);
    }

    /** Cut power to all actuators. Safe to call any time. */
    public void stop() {
        inOne.setPower(0);
        inTwo.setPower(0);
    }
}
