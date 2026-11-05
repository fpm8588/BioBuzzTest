// @ftc-toolchain generated: opmode — scaffolded; team edits expected
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// DISABLED (drive + dead wheels only):
// import org.firstinspires.ftc.teamcode.subsystems.Intake;
// import org.firstinspires.ftc.teamcode.subsystems.Limelight;
// import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RobotHealthMonitor;
import org.firstinspires.ftc.teamcode.util.Stats;

/*
 * Pedro Pathing autonomous driven by a finite state machine.
 * Field coordinates: 144x144 inches, (0,0) at the bottom-left corner,
 * heading in radians. Replace the poses below with ones for your routine
 * (the Pedro visualizer at https://visualizer.pedropathing.com helps).
 *
 * This is a test-build skeleton: it demonstrates driving to a pose, running the
 * shooter/intake for a timed scoring window, reading the Limelight, and parking.
 * Replace the scoring logic and poses once this season's field/game is known.
 */
@Autonomous(name = "Competition Auto", group = "Competition")
public class CompAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // DISABLED (drive + dead wheels only):
    // private Shooter shooter;
    // private Intake intake;
    // private Limelight limelight;
    private Stats stats;
    private RobotHealthMonitor health;

    private static final double SCORE_DURATION_SEC = 2.0;

    // TODO: replace with your real poses
    private final Pose startPose = new Pose(9, 72, Math.toRadians(0));
    private final Pose scorePose = new Pose(36, 72, Math.toRadians(0));
    private final Pose parkPose = new Pose(36, 36, Math.toRadians(90));

    private PathChain driveToScore, park;

    private void buildPaths() {
        driveToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(driveToScore);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // DISABLED (drive + dead wheels only): scoring window. Original code:
                    //   shooter.spinAtHigh(); intake.intakeIn(); setPathState(15);
                    // Drive straight on to the park pose instead.
                    follower.followPath(park, true);
                    setPathState(2);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > SCORE_DURATION_SEC) {
                    // shooter.stop(); // DISABLED (drive + dead wheels only)
                    // intake.stop();
                    stats.recordShot(true);
                    follower.followPath(park, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // DISABLED (drive + dead wheels only):
        // shooter = new Shooter(hardwareMap);
        // intake = new Intake(hardwareMap);
        // limelight = new Limelight(hardwareMap);
        stats = new Stats(telemetry);
        health = new RobotHealthMonitor(hardwareMap, telemetry, "CompAuto");
        Drawing.init();
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        setPathState(0);

        while (opModeIsActive()) {
            follower.update();
            // limelight.update(); // DISABLED (drive + dead wheels only)
            autonomousPathUpdate();
            stats.trackPose(follower.getPose());
            health.update();
            Drawing.drawDebug(follower);

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            // telemetry.addData("limelight target", limelight.hasTarget()); // DISABLED
            telemetry.update();
            stats.update();
        }

        // shooter.stop(); // DISABLED (drive + dead wheels only)
        // intake.stop();
        // limelight.stop();
        stats.writeSummaryToFile("CompAuto");
    }
}
