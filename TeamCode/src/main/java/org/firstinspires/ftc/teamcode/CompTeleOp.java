// @ftc-toolchain generated: teleop — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RobotHealthMonitor;
import org.firstinspires.ftc.teamcode.util.Stats;

/*
 * Competition TeleOp TeleOp. Controller bindings live in CompTeleOpControls.java
 * (edit that file to remap buttons). Behavior and automations live here.
 */
@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompTeleOp extends OpMode {

    private Shooter shooter;
    private Intake intake;
    private Lift lift;
    private Sorter sorter;
    private Follower follower;
    private boolean liftUpPrev;
    private boolean liftMidPrev;
    private boolean liftDownPrev;
    private boolean sortLeftPrev;
    private boolean sortRightPrev;
    private boolean slowActive;
    private boolean intakeInPrev;
    private Stats stats;
    private RobotHealthMonitor health;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        sorter = new Sorter(hardwareMap);
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
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        Gamepad driver = gamepad1, operator = gamepad2;

        // --- Drive ---
        double axial = CompTeleOpControls.driveForward(driver, operator);
        double lateral = CompTeleOpControls.driveStrafe(driver, operator);
        double yaw = CompTeleOpControls.driveTurn(driver, operator);
        slowActive = CompTeleOpControls.slowMode(driver, operator);
        double speed = slowActive ? 0.4 : 1.0;
        axial *= speed; lateral *= speed; yaw *= speed;
        follower.setTeleOpDrive(axial, lateral, yaw, false); // last arg: true = robot-centric
        follower.update();

        // --- Actions ---
        // group: shooter
        if (CompTeleOpControls.shooterHigh(driver, operator)) { shooter.spinAtHigh(); } // Shooter high-velocity preset: shooterHigh
        else if (CompTeleOpControls.shooterLow(driver, operator)) { shooter.spinAtLow(); } // Shooter low-velocity preset: shooterLow
        else { shooter.stop(); }
        // group: intake
        boolean intakeInNow = CompTeleOpControls.intakeIn(driver, operator);
        if (intakeInNow) { intake.intakeIn(); } // Intake in: intakeIn
        else if (CompTeleOpControls.intakeOut(driver, operator)) { intake.intakeOut(); } // Intake out / reject: intakeOut
        else { intake.stop(); }
        if (intakeInNow && !intakeInPrev) { stats.recordIntakeCycle(); }
        intakeInPrev = intakeInNow;
        // Lift to top position
        boolean liftUpNow = CompTeleOpControls.liftUp(driver, operator);
        if (liftUpNow && !liftUpPrev) { lift.goUp(); }
        liftUpPrev = liftUpNow;
        // Lift to middle position
        boolean liftMidNow = CompTeleOpControls.liftMid(driver, operator);
        if (liftMidNow && !liftMidPrev) { lift.goMid(); }
        liftMidPrev = liftMidNow;
        // Lift to bottom position
        boolean liftDownNow = CompTeleOpControls.liftDown(driver, operator);
        if (liftDownNow && !liftDownPrev) { lift.goDown(); }
        liftDownPrev = liftDownNow;
        // Route to left gate
        boolean sortLeftNow = CompTeleOpControls.sortLeft(driver, operator);
        if (sortLeftNow && !sortLeftPrev) { sorter.sortLeft(); }
        sortLeftPrev = sortLeftNow;
        // Route to right gate
        boolean sortRightNow = CompTeleOpControls.sortRight(driver, operator);
        if (sortRightNow && !sortRightPrev) { sorter.sortRight(); }
        sortRightPrev = sortRightNow;

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

        telemetry.addData("pose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
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
