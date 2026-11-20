package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Bench test for the Intake subsystem (gamepad1):
 *   a -> intakeIn() at Intake.INTAKE_RPM
 *   b -> intakeOut()
 *   back -> stop()
 * Shows the measured RPM, so you can check it reaches 900 (change Intake.INTAKE_RPM in Panels to test other speeds).
 */
@TeleOp(name = "Test Intake", group = "subsystems")
public class TestIntake extends OpMode {

    private Intake intake;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) intake.intakeIn();
        if (gamepad1.bWasPressed()) intake.intakeOut();
        if (gamepad1.backWasPressed()) intake.stop();

        String problem = intake.problem();
        if (problem != null) telemetry.addData("PROBLEM", problem);
        telemetry.addData("target RPM", "%.0f", Intake.INTAKE_RPM);
        telemetry.addData("measured RPM", "%.0f", intake.getRpm());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
    }
}
