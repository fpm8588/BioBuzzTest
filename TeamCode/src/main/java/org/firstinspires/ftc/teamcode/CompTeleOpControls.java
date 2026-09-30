// @ftc-toolchain generated: controls — scaffolded; driver edits expected
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Controller bindings for CompTeleOp. EDIT THIS FILE to remap controls —
 * change only which gamepad button/stick each action uses. No robot logic here.
 * driver = gamepad1, operator = gamepad2.
 */
public class CompTeleOpControls {

    // ===== DRIVE =====
    // forward is negative on the stick
    public static double driveForward(Gamepad driver, Gamepad operator) { return -driver.left_stick_y; }
    public static double driveStrafe(Gamepad driver, Gamepad operator) { return driver.left_stick_x; }
    public static double driveTurn(Gamepad driver, Gamepad operator) { return driver.right_stick_x; }

    // ===== SLOW MODE =====
    public static boolean slowMode(Gamepad driver, Gamepad operator) { return driver.left_trigger > 0.5; }

    // ===== ACTIONS =====
    // Shooter high-velocity preset
    public static boolean shooterHigh(Gamepad driver, Gamepad operator) { return operator.right_trigger > 0.1; }
    // Shooter low-velocity preset
    public static boolean shooterLow(Gamepad driver, Gamepad operator) { return operator.left_trigger > 0.1; }
    // Intake in
    public static boolean intakeIn(Gamepad driver, Gamepad operator) { return operator.dpad_up; }
    // Intake out / reject
    public static boolean intakeOut(Gamepad driver, Gamepad operator) { return operator.dpad_left; }
    // Lift to top position
    public static boolean liftUp(Gamepad driver, Gamepad operator) { return operator.x; }
    // Lift to middle position
    public static boolean liftMid(Gamepad driver, Gamepad operator) { return operator.a; }
    // Lift to bottom position
    public static boolean liftDown(Gamepad driver, Gamepad operator) { return operator.b; }
    // Route to left gate
    public static boolean sortLeft(Gamepad driver, Gamepad operator) { return operator.left_bumper; }
    // Route to right gate
    public static boolean sortRight(Gamepad driver, Gamepad operator) { return operator.right_bumper; }

    // ===== AUTOMATION TRIGGERS =====
    // Zero the robot's heading in the Pedro follower to the driver's current facing, for when the IMU heading drifts mid-match.
    public static boolean resetHeading(Gamepad driver, Gamepad operator) { return driver.a && driver.b; }
}
