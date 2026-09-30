// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Flywheel shooter with two velocity presets (test build; roles to be retuned when DECODE game pieces are known)
 */
@Configurable
public class Shooter {

    private DcMotorEx spinOne;
    private DcMotorEx spinTwo;

    // Hardware configuration names (must match the Driver Station config)
    public static final String SPIN_ONE_NAME = "spinOne";
    public static final String SPIN_TWO_NAME = "spinTwo";

    // --- Tunable constants (live-editable via the dashboard) ---
    public static double LOW_VELOCITY = 1100; // ticks/sec, close-range shot preset
    public static double HIGH_VELOCITY = 1400; // ticks/sec, far-range shot preset

    public Shooter(HardwareMap hardwareMap) {
        spinOne = hardwareMap.get(DcMotorEx.class, SPIN_ONE_NAME);
        spinOne.setDirection(DcMotorSimple.Direction.REVERSE);
        spinOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinTwo = hardwareMap.get(DcMotorEx.class, SPIN_TWO_NAME);
        spinTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        spinTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Spin both flywheels up to the close-range preset. */
    public void spinAtLow() {
        spinOne.setVelocity(LOW_VELOCITY);
        spinTwo.setVelocity(LOW_VELOCITY);
    }

    /** Spin both flywheels up to the far-range preset. */
    public void spinAtHigh() {
        spinOne.setVelocity(HIGH_VELOCITY);
        spinTwo.setVelocity(HIGH_VELOCITY);
    }

    /** Current average flywheel velocity, for telemetry/statistics. */
    public double getVelocity() {
        return (spinOne.getVelocity() + spinTwo.getVelocity()) / 2.0;
    }

    /** Cut power to all actuators. Safe to call any time. */
    public void stop() {
        spinOne.setPower(0);
        spinTwo.setPower(0);
    }
}
