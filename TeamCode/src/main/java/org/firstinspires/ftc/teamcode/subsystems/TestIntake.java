// @ftc-toolchain generated: bench-test — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Bench test for the Intake subsystem. Binds each action to a
 * gamepad1 button so you can exercise the subsystem in isolation on the robot.
 *
     *   a -> intakeIn()
     *   b -> intakeOut()
 *   back (always) -> stop()
 */
@Disabled // hidden: drive + dead wheels only build (remove to re-enable)
@TeleOp(name = "Test Intake", group = "subsystems")
public class TestIntake extends OpMode {

    private Intake intake;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            intake.intakeIn();
        }
        if (gamepad1.bWasPressed()) {
            intake.intakeOut();
        }

        if (gamepad1.backWasPressed()) {
            intake.stop();
        }

        telemetry.addData("subsystem", "Intake");
        telemetry.update();
    }
}
