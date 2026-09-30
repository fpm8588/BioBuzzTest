// @ftc-toolchain generated: bench-test — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Bench test for the Shooter subsystem. Binds each action to a
 * gamepad1 button so you can exercise the subsystem in isolation on the robot.
 *
     *   a -> spinAtLow()
     *   b -> spinAtHigh()
 *   back (always) -> stop()
 */
@TeleOp(name = "Test Shooter", group = "subsystems")
public class TestShooter extends OpMode {

    private Shooter shooter;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            shooter.spinAtLow();
        }
        if (gamepad1.bWasPressed()) {
            shooter.spinAtHigh();
        }

        if (gamepad1.backWasPressed()) {
            shooter.stop();
        }

        telemetry.addData("subsystem", "Shooter");
        telemetry.update();
    }
}
