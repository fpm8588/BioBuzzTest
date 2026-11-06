// @ftc-toolchain generated: bench-test — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Bench test for the Sorter subsystem. Binds each action to a
 * gamepad1 button so you can exercise the subsystem in isolation on the robot.
 *
     *   a -> sortLeft()
     *   b -> sortRight()
     *   x -> neutral()
 *   back (always) -> stop()
 */
@Disabled // hidden: drive + dead wheels only build (remove to re-enable)
@TeleOp(name = "Test Sorter", group = "subsystems")
public class TestSorter extends OpMode {

    private Sorter sorter;

    @Override
    public void init() {
        sorter = new Sorter(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            sorter.sortLeft();
        }
        if (gamepad1.bWasPressed()) {
            sorter.sortRight();
        }
        if (gamepad1.xWasPressed()) {
            sorter.neutral();
        }

        if (gamepad1.backWasPressed()) {
            sorter.stop();
        }

        telemetry.addData("subsystem", "Sorter");
        telemetry.update();
    }
}
