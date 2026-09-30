// @ftc-toolchain generated: bench-test — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Bench test for the Lift subsystem. Binds each action to a
 * gamepad1 button so you can exercise the subsystem in isolation on the robot.
 *
     *   a -> goUp()
     *   b -> goMid()
     *   x -> goDown()
 *   back (always) -> stop()
 */
@TeleOp(name = "Test Lift", group = "subsystems")
public class TestLift extends OpMode {

    private Lift lift;

    @Override
    public void init() {
        lift = new Lift(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            lift.goUp();
        }
        if (gamepad1.bWasPressed()) {
            lift.goMid();
        }
        if (gamepad1.xWasPressed()) {
            lift.goDown();
        }

        if (gamepad1.backWasPressed()) {
            lift.stop();
        }

        telemetry.addData("subsystem", "Lift");
        telemetry.update();
    }
}
