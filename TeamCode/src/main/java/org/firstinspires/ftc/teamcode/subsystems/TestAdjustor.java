// @ftc-toolchain generated: bench-test — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Bench test for the Adjustor subsystem. Binds each action to a
 * gamepad1 button so you can exercise the subsystem in isolation on the robot.
 *
 *   a -> goLow()
 *   b -> goMid()
 *   y -> goHigh()
 *   dpad up / dpad down -> nudge the position up / down by 0.02
 */
@TeleOp(name = "Test Adjustor", group = "subsystems")
public class TestAdjustor extends OpMode {

    private Adjustor adjustor;

    @Override
    public void init() {
        adjustor = new Adjustor(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            adjustor.goLow();
        }
        if (gamepad1.bWasPressed()) {
            adjustor.goMid();
        }
        if (gamepad1.yWasPressed()) {
            adjustor.goHigh();
        }
        if (gamepad1.dpadUpWasPressed()) {
            adjustor.nudge(0.02);
        }
        if (gamepad1.dpadDownWasPressed()) {
            adjustor.nudge(-0.02);
        }

        if (!adjustor.isEnabled()) {
            telemetry.addLine("DISABLED: AuxSettings.ENABLED is false, so this does nothing and needs no hardware.");
        }
        telemetry.addData("subsystem", "Adjustor");
        telemetry.addData("position (last commanded)", adjustor.getPosition());
        telemetry.update();
    }
}
