package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * Dead-wheel check. The motors are floated and never driven, so push the robot around by hand.
 * It shows Pedro's pose next to the raw pod counts so you can check directions, heading and
 * offsets, and work out ticks-to-inches. See docs/HARDWARE_CONFIG.md and docs/ODOMETRY.md.
 *
 *   A          zero the pose and the raw counts (do this before each test)
 *   D-pad up/down   change the real distance you pushed (inches, default 48)
 */
@TeleOp(name = "Odometry Test (push by hand)", group = "Odometry")
public class OdometryTest extends LinearOpMode {

    // Config names of the ports the pods plug into (must match Constants.java).
    private static final String FORWARD_POD_PORT = "rf";
    private static final String STRAFE_POD_PORT = "rb";
    private static final String[] DRIVE_MOTORS = {"lf", "lb", "rf", "rb"};

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Let the wheels roll freely so the robot can be pushed by hand.
        for (String name : DRIVE_MOTORS) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        DcMotorEx forwardPod = hardwareMap.get(DcMotorEx.class, FORWARD_POD_PORT);
        DcMotorEx strafePod = hardwareMap.get(DcMotorEx.class, STRAFE_POD_PORT);

        int forwardBase = forwardPod.getCurrentPosition();
        int strafeBase = strafePod.getCurrentPosition();
        double realInches = 48.0;
        boolean upPrev = false, downPrev = false, aPrev = false;

        telemetry.addLine("Push the robot by hand. Press A to zero.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            boolean a = gamepad1.a, up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if (a && !aPrev) {
                follower.setPose(new Pose(0, 0, 0));
                forwardBase = forwardPod.getCurrentPosition();
                strafeBase = strafePod.getCurrentPosition();
            }
            if (up && !upPrev) realInches += 1;
            if (down && !downPrev) realInches = Math.max(1, realInches - 1);
            aPrev = a; upPrev = up; downPrev = down;

            Pose pose = follower.getPose();
            int forwardTicks = forwardPod.getCurrentPosition() - forwardBase;
            int strafeTicks = strafePod.getCurrentPosition() - strafeBase;

            telemetry.addLine("--- Pedro pose (inches / degrees) ---");
            telemetry.addData("x (forward +)", "%.2f", pose.getX());
            telemetry.addData("y (left +)", "%.2f", pose.getY());
            telemetry.addData("heading (left turn +)", "%.1f", Math.toDegrees(pose.getHeading()));
            telemetry.addLine("--- Raw pod counts since zero ---");
            telemetry.addData("forward pod ticks", forwardTicks);
            telemetry.addData("strafe pod ticks", strafeTicks);
            telemetry.addLine("--- Ticks to inches ---");
            telemetry.addData("real distance pushed (in)", "%.0f  (D-pad up/down)", realInches);
            if (Math.abs(forwardTicks) > 200) {
                telemetry.addData("forward ticksToInches", "%.6f", realInches / Math.abs(forwardTicks));
            }
            if (Math.abs(strafeTicks) > 200) {
                telemetry.addData("strafe ticksToInches", "%.6f", realInches / Math.abs(strafeTicks));
            }
            telemetry.addLine("--- Expect ---");
            telemetry.addLine("Push FORWARD: x rises. Push LEFT: y rises. Turn LEFT: heading rises.");
            telemetry.addLine("Wrong way? Flip that pod's Encoder direction in Constants.java.");
            telemetry.addLine("Spin in place: x and y should barely move.");
            telemetry.update();
        }
    }
}
