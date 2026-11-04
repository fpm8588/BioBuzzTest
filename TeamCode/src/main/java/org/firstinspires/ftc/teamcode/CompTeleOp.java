// @ftc-toolchain generated: teleop — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Adjustor;
import org.firstinspires.ftc.teamcode.subsystems.GoalTracker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
// DISABLED (sorter hardware not set up yet): re-enable with the sorter code below.
// import org.firstinspires.ftc.teamcode.subsystems.Sorter;
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
    private Adjustor adjustor;
    private GoalTracker goal;
    // DISABLED (sorter hardware not set up yet):
    // private Sorter sorter;
    private Follower follower;
    private boolean adjustorHighPrev;
    private boolean adjustorMidPrev;
    private boolean adjustorLowPrev;
    // private boolean sortLeftPrev;
    // private boolean sortRightPrev;
    private boolean slowActive;
    private boolean intakeInPrev;
    private Stats stats;
    private RobotHealthMonitor health;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        adjustor = new Adjustor(hardwareMap);
        goal = GoalTracker.create(hardwareMap); // works without a Limelight in the configuration: aiming is just off
        // sorter = new Sorter(hardwareMap); // DISABLED (sorter hardware not set up yet)
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
        goal.update();
        boolean aiming = CompTeleOpControls.aimAtGoal(driver, operator) && goal.hasGoal();
        if (aiming) yaw = goal.getAimTurn(); // the aim takes over the turning only; the driver still translates
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
        // Shot-angle adjustor to its high position
        boolean adjustorHighNow = CompTeleOpControls.adjustorHigh(driver, operator);
        if (adjustorHighNow && !adjustorHighPrev) { adjustor.goHigh(); }
        adjustorHighPrev = adjustorHighNow;
        // Shot-angle adjustor to its middle position
        boolean adjustorMidNow = CompTeleOpControls.adjustorMid(driver, operator);
        if (adjustorMidNow && !adjustorMidPrev) { adjustor.goMid(); }
        adjustorMidPrev = adjustorMidNow;
        // Shot-angle adjustor to its low position
        boolean adjustorLowNow = CompTeleOpControls.adjustorLow(driver, operator);
        if (adjustorLowNow && !adjustorLowPrev) { adjustor.goLow(); }
        adjustorLowPrev = adjustorLowNow;
        // DISABLED (sorter hardware not set up yet):
        // // Route to left gate
        // boolean sortLeftNow = CompTeleOpControls.sortLeft(driver, operator);
        // if (sortLeftNow && !sortLeftPrev) { sorter.sortLeft(); }
        // sortLeftPrev = sortLeftNow;
        // // Route to right gate
        // boolean sortRightNow = CompTeleOpControls.sortRight(driver, operator);
        // if (sortRightNow && !sortRightPrev) { sorter.sortRight(); }
        // sortRightPrev = sortRightNow;

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

        telemetry.addData("shooter velocity (ticks/sec)", "%.0f", shooter.getVelocity());
        telemetry.addData("adjustor", adjustor.getPosition());
        telemetry.addData("goal tag", goal.status() + (aiming ? (goal.isAimed() ? "  [AIMED]" : "  [AIMING]") : ""));
        telemetry.addData("pose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        goal.stop();
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
