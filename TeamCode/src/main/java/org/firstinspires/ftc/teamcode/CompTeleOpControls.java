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
    // Pedro's own teleop mapping: forward is negative on the stick; Pedro's strafe is positive to the LEFT but the stick is positive to
    // the RIGHT; and a positive turn is counter-clockwise (left) but the stick is positive to the right. So all three are negated.
    // (The motor directions in Constants.java are set so that the robot then drives the way the stick points.)
    public static double driveForward(Gamepad driver, Gamepad operator) { return -driver.left_stick_y; }
    public static double driveStrafe(Gamepad driver, Gamepad operator) { return -driver.left_stick_x; }

    /** How much of the stick the turn uses: 1.0 = full, lower = gentler (it is 0.8; it was 1.0). Slow mode still multiplies this by 0.4. */
    public static double TURN_SENSITIVITY = 0.8;
    public static double driveTurn(Gamepad driver, Gamepad operator) { return -driver.right_stick_x * TURN_SENSITIVITY; }

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
