// @ftc-toolchain generated: teleop — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shoot;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RobotHealthMonitor;
import org.firstinspires.ftc.teamcode.util.Stats;

import java.util.List;

/*
 * Competition TeleOp TeleOp. Controller bindings live in CompTeleOpControls.java
 * (edit that file to remap buttons). Behavior and automations live here.
 */
@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompTeleOp extends OpMode {

    private Shoot shoot;
    private Intake intake;
    private Follower follower;
    private boolean slowActive;
    private boolean intakeInPrev;
    private Stats stats;
    private RobotHealthMonitor health;
    private List<LynxModule> hubs;
    private double lastLoopSec;
    private double loopMs;

    @Override
    public void init() {
        // Bulk reads, as in the Tuning OpModes: each hub sends all its encoder and velocity data in ONE round trip per loop (cleared at the
        // top of loop()). Without this every encoder, velocity and IMU read is its own round trip and the loop gets slow.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        shoot = new Shoot(hardwareMap);
        intake = new Intake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        stats = new Stats(telemetry);
        health = new RobotHealthMonitor(hardwareMap, telemetry, "CompTeleOp");
        Drawing.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        follower.startTeleopDrive();
        lastLoopSec = System.nanoTime() / 1e9;
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        double nowSec = System.nanoTime() / 1e9;
        loopMs = 0.9 * loopMs + 0.1 * (nowSec - lastLoopSec) * 1000.0;
        lastLoopSec = nowSec;

        Gamepad driver = gamepad1, operator = gamepad2;

        // --- Drive ---
        double axial = CompTeleOpControls.driveForward(driver, operator);
        double lateral = CompTeleOpControls.driveStrafe(driver, operator);
        double yaw = CompTeleOpControls.driveTurn(driver, operator);
        slowActive = CompTeleOpControls.slowMode(driver, operator);
        double speed = slowActive ? 0.4 : 1.0;
        axial *= speed; lateral *= speed; yaw *= speed;
        follower.setTeleOpDrive(axial, lateral, yaw, !CompTeleOpControls.FIELD_CENTRIC); // last arg: true = robot-centric (see CompTeleOpControls.FIELD_CENTRIC)
        follower.update();

        // --- Actions ---
        // Shoot: held at Shoot.SHOOT_RPM (2100) while the button is held
        if (CompTeleOpControls.shoot(driver, operator)) { shoot.spin(); } // Shoot: shoot
        else { shoot.stop(); }
        // Intake: held at Intake.INTAKE_RPM (900), in or out
        boolean intakeInNow = CompTeleOpControls.intakeIn(driver, operator);
        if (intakeInNow) { intake.intakeIn(); } // Intake in: intakeIn
        else if (CompTeleOpControls.intakeOut(driver, operator)) { intake.intakeOut(); } // Intake out / reject: intakeOut
        else { intake.stop(); }
        if (intakeInNow && !intakeInPrev) { stats.recordIntakeCycle(); }
        intakeInPrev = intakeInNow;

        // --- Automations ---
        if (CompTeleOpControls.resetHeading(driver, operator)) resetHeading();

        // --- Statistics ---
        // NOTE: recordShot() has no automatic trigger yet — there's no sensor telling us
        // whether a shot scored. Wire it to a scoring sensor or a manual driver confirm
        // once this season's scoring mechanism is known.
        stats.trackPose(follower.getPose());
        stats.update();
        health.update(driver);
        Drawing.drawRobot(follower.getPose());

        String shootProblem = shoot.problem(), intakeProblem = intake.problem();
        if (shootProblem != null) telemetry.addData("SHOOT PROBLEM", shootProblem);
        if (intakeProblem != null) telemetry.addData("INTAKE PROBLEM", intakeProblem);
        telemetry.addData("shoot RPM", "%.0f (target %.0f)%s", shoot.getRpm(), Shoot.SHOOT_RPM, shoot.isAtSpeed() ? "  READY" : "");
        telemetry.addData("intake RPM", "%.0f (target %.0f)", intake.getRpm(), Intake.INTAKE_RPM);
        telemetry.addData("drive mode", CompTeleOpControls.FIELD_CENTRIC ? "FIELD-centric" : "ROBOT-centric");
        telemetry.addData("loop", "%.0f ms (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);
        telemetry.addData("driver sticks (lx ly rx)", "%.2f %.2f %.2f", driver.left_stick_x, driver.left_stick_y, driver.right_stick_x);
        telemetry.addData("pose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void stop() {
        shoot.stop();
        intake.stop();
        stats.writeSummaryToFile("CompTeleOp");
    }

    /**
     * Zero the robot's heading in the Pedro follower to the driver's current facing, for when the IMU heading drifts mid-match.
     */
    private void resetHeading() {
        Pose current = follower.getPose();
        follower.setPose(new Pose(current.getX(), current.getY(), 0));
    }
}
