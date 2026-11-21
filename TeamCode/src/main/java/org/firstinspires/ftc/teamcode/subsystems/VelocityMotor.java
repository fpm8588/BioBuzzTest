package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * One motor run at a speed in RPM (the motor controller holds the speed with its own velocity control; the encoder must be plugged in).
 * Shoot and Intake are built on this. The SDK works in encoder ticks per second, so RPM is converted with the motor's ticks per
 * revolution: taken from the motor type chosen in the Driver Hub configuration, unless an override is given.
 */
final class VelocityMotor {

    private final DcMotorEx motor;
    private final double ticksPerRev;
    private final double maxRpm;

    /**
     * @param ticksPerRevOverride encoder ticks per revolution of the motor shaft to use; 0 or less = use the motor type set in the configuration
     * @param floatWhenStopped    true = the motor coasts when stopped (flywheels), false = it brakes
     */
    VelocityMotor(HardwareMap hardwareMap, String name, boolean reversed, double ticksPerRevOverride, boolean floatWhenStopped) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(floatWhenStopped ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // velocity control needs the encoder
        MotorConfigurationType type = motor.getMotorType();
        ticksPerRev = ticksPerRevOverride > 0 ? ticksPerRevOverride : (type == null ? 0 : type.getTicksPerRev());
        maxRpm = type == null ? 0 : type.getMaxRPM() * type.getAchieveableMaxRPMFraction();
    }

    /** null if all is well, else what is wrong (shown on the screen). */
    String problem(double requestedRpm) {
        if (ticksPerRev <= 0) {
            return "no ticks per revolution known: choose this motor's type in the Driver Hub configuration (or set TICKS_PER_REV in the code)";
        }
        if (maxRpm > 0 && Math.abs(requestedRpm) > maxRpm) {
            return String.format("%.0f RPM is more than this motor can reach (about %.0f RPM)", Math.abs(requestedRpm), maxRpm);
        }
        return null;
    }

    /** Run at this speed (negative = the other way). Does nothing, and stops the motor, if the ticks per revolution are not known. */
    void setRpm(double rpm) {
        if (ticksPerRev <= 0) {
            motor.setPower(0);
            return;
        }
        motor.setVelocity(rpm * ticksPerRev / 60.0);
    }

    /** Measured speed in RPM (negative = running the other way). 0 if the ticks per revolution are not known. */
    double getRpm() {
        return ticksPerRev <= 0 ? 0 : motor.getVelocity() * 60.0 / ticksPerRev;
    }

    void stop() {
        motor.setPower(0);
    }
}
