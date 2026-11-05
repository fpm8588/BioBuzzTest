package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Adjustor;
import org.firstinspires.ftc.teamcode.subsystems.GoalTracker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Drawing;

/*
 * SHOOT ON TAGS: drives to the spot where the shooter can score into the CELLs above the alliance's AprilTag strip, and shoots. Nothing else:
 * no obstacle avoidance and no pollen chasing.
 *
 * WHERE: the strip hangs under the CELLs on the HIVE. The shooting position is worked out from the HIVE's size and position (ShootSettings:
 * the frame is about 49 x 39 in centred on the field, which face the strip is on, how far along it, and how far from it to stand) by
 * util/ShootPlan: the robot stands SHOOT_DISTANCE_IN (camera to strip) out from the face, looking straight at the strip. The route from
 * the start goes straight there if that is clear, otherwise round the HIVE (frame plus HIVE_ROUTE_MARGIN_IN). The BLUE alliance's plan is the
 * RED one turned 180 degrees (LeaveAndPark.ROTATE_180, like every other autonomous).
 *
 * THEN: (1) the robot has been spinning the shooter up since it started driving; (2) if USE_VISION and a tag of the strip is in view, it turns to
 * face the strip centre and creeps to the right range (never further than MAX_CORRECTION_IN from the planned position); with no tag in view it
 * skips this and shoots from where it is; (3) the intake feeds for FEED_SEC; (4) everything stops.
 *
 * The camera is mounted low and level for pollen and the tags are high up: it probably cannot see them from the shooting position (the INIT
 * screen says so and what tilt would fix it). The robot is then positioned by odometry alone, so measure the HIVE and the start. Run
 * "Shoot On Tags Test" first. See docs/SHOOT_ON_TAGS.md.
 *
 * Needs the drive motors, dead wheels and IMU, the intake, shoot1/shoot2 and adjustor, and optionally the Limelight (without it, no fine positioning).
 */
@Autonomous(name = "Shoot On Tags", group = "Competition")
public class ShootOnTags extends LinearOpMode {

    private enum State { DELAY, DRIVE, SETTLE, VISION, SHOOT, DONE, ABORTED }

    private Follower follower;
    private State state = State.DELAY;
    private double stateSince;
    private String note = "";

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        Drawing.init();
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Adjustor adjustor = new Adjustor(hardwareMap);
        GoalTracker goal = GoalTracker.create(hardwareMap);   // the alliance's strip follows LeaveAndPark.ROTATE_180; missing Limelight is fine

        ShootSetup setup = ShootSetup.build();
        try {
            // Show the plan while waiting, and rebuild it so a value changed in Panels shows up straight away.
            while (opModeInInit()) {
                setup = ShootSetup.build();
                goal.update();
                telemetry.addLine("SHOOT ON TAGS: drive to the shooting position under the CELLs and shoot.");
                setup.show(telemetry);
                telemetry.addData("camera", ShootSettings.USE_VISION ? goal.status() : "vision off (USE_VISION = false)");
                telemetry.update();
                sleep(100);
            }
            if (isStopRequested()) return;

            setup = ShootSetup.build();   // everything again after START so the latest Panels values are used
            if (!setup.blocking.isEmpty()) {
                while (opModeIsActive()) {   // do nothing, just explain
                    setup.show(telemetry);
                    telemetry.update();
                    sleep(200);
                }
                return;
            }

            final ShootSetup plan = setup;
            follower.setStartingPose(plan.start);
            final PathChain chain = plan.buildChain(follower);
            final double power = Math.max(0.1, Math.min(1.0, ShootSettings.MAX_POWER));
            final Pose shootPose = plan.goal;
            ElapsedTime run = new ElapsedTime();
            double spinStart = -1, feedStart = -1;
            boolean teleopDriving = false;   // true once the follower was switched to hand-driving (fine positioning)

            while (opModeIsActive()) {
                follower.update();
                goal.update();
                Pose pose = follower.getPose();
                double now = run.seconds();
                double inState = now - stateSince;

                switch (state) {
                    case DELAY:
                        if (now >= ShootSettings.START_DELAY_SEC) {
                            adjustor.setPosition(ShootSettings.ADJUSTOR_POSITION);
                            if (ShootSettings.USE_HIGH_SHOT) shooter.spinAtHigh(); else shooter.spinAtLow();
                            spinStart = now;
                            if (chain == null) {
                                enter(State.SETTLE, now);
                            } else {
                                follower.followPath(chain, power, true);
                                enter(State.DRIVE, now);
                            }
                        }
                        break;

                    case DRIVE:
                        if (!follower.isBusy()) enter(State.SETTLE, now);
                        break;

                    case SETTLE:   // let the robot come to rest at the shooting position
                        if (inState >= 0.3) enter(ShootSettings.USE_VISION && goal.available() ? State.VISION : State.SHOOT, now);
                        break;

                    case VISION: {
                        if (!goal.hasGoal()) {
                            if (inState > AprilTagSettings.AUTO_NO_TAG_SKIP_SEC) {
                                note = "no tag in view: shooting from the planned position";
                                if (teleopDriving) holdStill();
                                enter(State.SHOOT, now);
                            }
                            break;
                        }
                        double range = goal.getDistanceIn();
                        double error = Double.isNaN(range) ? 0 : range - ShootSettings.SHOOT_DISTANCE_IN;   // + = too far from the strip
                        double forward = Math.abs(error) <= ShootSettings.RANGE_TOLERANCE_IN ? 0
                                : Math.max(-ShootSettings.MAX_CORRECT_POWER, Math.min(ShootSettings.MAX_CORRECT_POWER, ShootSettings.K_RANGE * error));
                        double moved = Math.hypot(pose.getX() - shootPose.getX(), pose.getY() - shootPose.getY());
                        if (moved >= ShootSettings.MAX_CORRECTION_IN) {
                            double away = (pose.getX() - shootPose.getX()) * Math.cos(pose.getHeading()) + (pose.getY() - shootPose.getY()) * Math.sin(pose.getHeading());
                            if (away * forward > 0) forward = 0;   // already as far as vision may take it, and still wants to go further the same way
                        }
                        if (!teleopDriving) {
                            follower.startTeleopDrive(true);
                            teleopDriving = true;
                        }
                        follower.setTeleOpDrive(forward, 0, goal.getAimTurn(), true);
                        note = String.format("fine positioning: %.1f deg to turn, range error %s", goal.getAimErrorDeg(), Double.isNaN(range) ? "?" : String.format("%+.1f in", error));
                        if ((goal.isAimed() && forward == 0) || inState > ShootSettings.VISION_TIMEOUT_SEC) {
                            holdStill();
                            enter(State.SHOOT, now);
                        }
                        break;
                    }

                    case SHOOT:
                        if (now - spinStart >= ShootSettings.SPINUP_SEC) {
                            if (feedStart < 0) feedStart = now;
                            intake.intakeIn();
                            note = "feeding";
                            if (now - feedStart >= ShootSettings.FEED_SEC) {
                                shooter.stop();
                                intake.stop();
                                note = "shot done";
                                enter(State.DONE, now);
                            }
                        } else {
                            note = String.format("shooter spinning up, %.1f s to go", ShootSettings.SPINUP_SEC - (now - spinStart));
                        }
                        break;

                    case DONE:
                    case ABORTED:
                        break;
                }

                // ---- safety net: never touch the walls or the HIVE, never wander off the route, never run past the time limit
                if (state == State.DRIVE || state == State.VISION) {
                    String reason = plan.guard.wallViolation(pose.getX(), pose.getY(), pose.getHeading());
                    if (reason == null) reason = plan.guard.keepOutViolation(pose.getX(), pose.getY(), pose.getHeading());
                    if (reason == null && state == State.DRIVE && plan.offRoute(pose.getX(), pose.getY()) > ShootSettings.MAX_OFF_PATH_IN) {
                        reason = "robot is more than " + ShootSettings.MAX_OFF_PATH_IN + " in off the route";
                    }
                    if (reason != null) abort(reason, shooter, intake);
                }
                if (state != State.DONE && state != State.ABORTED && state != State.DELAY && now - ShootSettings.START_DELAY_SEC > ShootSettings.TIMEOUT_SEC) {
                    abort("took longer than " + ShootSettings.TIMEOUT_SEC + " s", shooter, intake);
                }

                telemetry.addData("state", state + (note.isEmpty() ? "" : ": " + note));
                telemetry.addData("pose", "(%.1f, %.1f) %.0f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
                telemetry.addData("shooting position", "(%.1f, %.1f) %.0f deg", shootPose.getX(), shootPose.getY(), Math.toDegrees(shootPose.getHeading()));
                telemetry.addData("off the route (in)", "%.1f", plan.offRoute(pose.getX(), pose.getY()));
                telemetry.addData("camera", ShootSettings.USE_VISION ? goal.status() : "vision off");
                telemetry.addData("shooter velocity (ticks/sec)", "%.0f", shooter.getVelocity());
                telemetry.update();
                Drawing.drawDebug(follower);
            }
        } finally {
            shooter.stop();
            intake.stop();
            follower.breakFollowing();   // motors off
            follower.setMaxPower(1.0);   // the follower settings are shared with the other OpModes
            goal.stop();
        }
    }

    private void enter(State next, double now) {
        state = next;
        stateSince = now;
    }

    private void abort(String reason, Shooter shooter, Intake intake) {
        state = State.ABORTED;
        note = reason;
        shooter.stop();
        intake.stop();
        holdStill();
    }

    /** Stop with the motors braking (breakFollowing would let them float and the robot coast). */
    private void holdStill() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }
}
