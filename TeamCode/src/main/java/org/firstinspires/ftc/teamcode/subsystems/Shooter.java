// @ftc-toolchain generated: subsystem — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Two-wheel flywheel shooter (shoot1 and shoot2 face each other, so they spin in opposite directions), run on velocity
 * (the motor controller holds the speed), not on power. Works the same from any OpMode, autonomous or teleop: create it
 * with the hardware map, call spinAtLow() / spinAtHigh() / stop().
 */
@Configurable
public class Shooter {

    private DcMotorEx shootOne;
    private DcMotorEx shootTwo;
    private final boolean enabled;   // AuxSettings.ENABLED when created: false = no hardware, every command does nothing

    // Hardware configuration names (must match the Driver Station config)
    public static final String SHOOT_ONE_NAME = "shoot1";
    public static final String SHOOT_TWO_NAME = "shoot2";

    // --- Tunable constants (live-editable via the dashboard) ---
    public static double LOW_VELOCITY = 1100; // ticks/sec, close-range shot preset
    public static double HIGH_VELOCITY = 1400; // ticks/sec, far-range shot preset

    /**
     * The two wheels face each other, so one motor is reversed relative to the other and both run at a POSITIVE velocity to
     * throw the game piece out. If a wheel throws the wrong way, flip its flag here (read when the Shooter is created).
     */
    public static boolean SHOOT_ONE_REVERSED = true;
    public static boolean SHOOT_TWO_REVERSED = false;

    public Shooter(HardwareMap hardwareMap) {
        enabled = AuxSettings.ENABLED;
        if (!enabled) return;
        shootOne = hardwareMap.get(DcMotorEx.class, SHOOT_ONE_NAME);
        shootOne.setDirection(SHOOT_ONE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shootOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // velocity control needs the encoder
        shootTwo = hardwareMap.get(DcMotorEx.class, SHOOT_TWO_NAME);
        shootTwo.setDirection(SHOOT_TWO_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shootTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** false when AuxSettings.ENABLED was off at creation: nothing is connected and nothing moves. */
    public boolean isEnabled() {
        return enabled;
    }

    /** Spin both flywheels up to the close-range preset. */
    public void spinAtLow() {
        setVelocity(LOW_VELOCITY);
    }

    /** Spin both flywheels up to the far-range preset. */
    public void spinAtHigh() {
        setVelocity(HIGH_VELOCITY);
    }

    /** Spin both flywheels at any velocity, ticks/sec (positive = throw the game piece out). */
    public void setVelocity(double ticksPerSec) {
        if (!enabled) return;
        shootOne.setVelocity(ticksPerSec);
        shootTwo.setVelocity(ticksPerSec);
    }

    /** Current average flywheel velocity, ticks/sec, for telemetry/statistics. */
    public double getVelocity() {
        if (!enabled) return 0;
        return (shootOne.getVelocity() + shootTwo.getVelocity()) / 2.0;
    }

    /** Cut power to all actuators. Safe to call any time. */
    public void stop() {
        if (!enabled) return;
        shootOne.setPower(0);
        shootTwo.setPower(0);
    }
}
