package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Bench test for the Shoot subsystem (gamepad1):
 *   a -> spin() to Shoot.SHOOT_RPM
 *   b or back -> stop()
 * Shows the measured RPM, so you can check it reaches 2100 (change Shoot.SHOOT_RPM in Panels to test other speeds).
 */
@TeleOp(name = "Test Shoot", group = "subsystems")
public class TestShoot extends OpMode {

    private Shoot shoot;

    @Override
    public void init() {
        shoot = new Shoot(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) shoot.spin();
        if (gamepad1.bWasPressed() || gamepad1.backWasPressed()) shoot.stop();

        String problem = shoot.problem();
        if (problem != null) telemetry.addData("PROBLEM", problem);
        telemetry.addData("target RPM", "%.0f", Shoot.SHOOT_RPM);
        telemetry.addData("measured RPM", "%.0f", shoot.getRpm());
        telemetry.addData("up to speed", shoot.isAtSpeed());
        telemetry.update();
    }

    @Override
    public void stop() {
        shoot.stop();
    }
}
