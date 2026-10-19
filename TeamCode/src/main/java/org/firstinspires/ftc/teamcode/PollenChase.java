package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;
import org.firstinspires.ftc.teamcode.util.PollenMath;

/*
 * Pollen Chase: finds a yellow pollen with the Limelight, turns to face it, and drives up to it, stopping when it is
 * PollenSettings.STOP_DISTANCE_IN from the camera. It steers straight from what the camera sees (visual servoing), so
 * it does not depend on the pollen's field position. Odometry is only used for the safety checks.
 *
 * States: SEARCH (spin in place until a pollen has been seen for ACQUIRE_TIME_SEC) -> CHASE (turn and drive) ->
 * ARRIVED. If the pollen is lost for LOST_TIMEOUT_SEC it searches again, turning toward where it was last seen.
 * Nothing is picked up: this branch has no intake. When the intake is back, start it in ARRIVED.
 *
 * Safety: every loop the robot's footprint is checked against the field walls, the HIVE and the FLOWER keep-out areas
 * (the same guard and numbers as Leave and Park, including the robot size; ENABLE_FIELD_GUARD turns this part off for
 * bench tests), and against MAX_TRAVEL_IN from the start. If any fails, or RUN_TIMEOUT_SEC passes, the motors stop.
 * Before moving it also checks that the robot can spin in place at the start pose without touching anything, and if the
 * Limelight pipeline is not confirmed it does not move. Start on open floor, not against a wall.
 *
 * Needs: the drive motors, dead wheels and IMU (see HARDWARE_CONFIG.md) plus the Limelight named "limelight".
 * Setup, camera numbers and tuning: docs/LIMELIGHT_POLLEN.md. Run Pollen Vision Test first.
 * First drive: PollenSettings.CHASE_MAX_POWER = 0.3, robot on the field with room, one pollen, hand on STOP.
 */
@Autonomous(name = "Pollen Chase", group = "Competition")
public class PollenChase extends LinearOpMode {

    private enum State { SEARCH, CHASE, ARRIVED, NOT_FOUND, ABORTED }

    private static final double STICKY_DEG = 5.0; // keep following the same ball if it moves less than this between frames

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Drawing.init();
        Limelight limelight = new Limelight(hardwareMap);
        boolean pipelineOk = limelight.selectPipeline(PollenSettings.POLLEN_PIPELINE, 3000);
        final LeaveParkPlan guard = LeaveAndPark.buildPlan();

        Pose start = new Pose(PollenSettings.START_X, PollenSettings.START_Y, Math.toRadians(PollenSettings.START_HEADING_DEG));
        // The robot may spin in place while searching, so no heading at the start pose may touch anything.
        String startProblem = null;
        if (PollenSettings.ENABLE_FIELD_GUARD) {
            for (int deg = 0; deg < 360 && startProblem == null; deg += 15) {
                String v = guard.violation(start.getX(), start.getY(), Math.toRadians(deg));
                if (v != null) startProblem = v + " if it turns to " + deg + " deg here: start further from it";
            }
        }

        // Preview while waiting: what the camera sees right now, so a bad pipeline shows before the match.
        while (opModeInInit()) {
            limelight.update();
            Limelight.Sighting s = limelight.getPollen(PollenSettings.MIN_AREA, PollenSettings.MAX_STALENESS_MS, false, 0, STICKY_DEG);
            telemetry.addData("Limelight", limelight.statusLine());
            telemetry.addData("pipeline " + PollenSettings.POLLEN_PIPELINE, pipelineOk ? "selected" : "NOT confirmed: the robot will not move");
            telemetry.addData("start pose", "(%.1f, %.1f) facing %.0f deg%s", start.getX(), start.getY(),
                    PollenSettings.START_HEADING_DEG, startProblem == null ? "" : "  PROBLEM: " + startProblem);
            telemetry.addData("pollen", s == null ? "none in view" : String.format("tx %.1f  ty %.1f  area %.3f%%  dist %.1f in",
                    s.txDeg, s.tyDeg, s.area * 100, PollenSettings.distanceIn(s.tyDeg)));
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) {
            limelight.stop();
            return;
        }

        follower.setStartingPose(start);
        PollenMath.Gains gains = PollenSettings.gains();
        double searchPower = Math.min(0.5, Math.abs(PollenSettings.SEARCH_TURN_POWER));
        double searchDir = PollenSettings.SEARCH_TURN_POWER >= 0 ? 1 : -1;

        State state = State.SEARCH;
        String note = "";
        if (!pipelineOk) {
            state = State.ABORTED;
            note = "Limelight pipeline " + PollenSettings.POLLEN_PIPELINE + " was not confirmed";
        } else if (startProblem != null) {
            state = State.ABORTED;
            note = "start pose: " + startProblem;
        }

        boolean haveLast = false;
        double lastTx = 0;
        double seenSince = -1;
        double lastSeenAt = -1;
        double lastDistance = Double.NaN;
        ElapsedTime run = new ElapsedTime();
        ElapsedTime stateTimer = new ElapsedTime();

        try {
            follower.setMaxPower(Math.max(0.1, Math.min(1.0, PollenSettings.CHASE_MAX_POWER)));
            follower.startTeleopDrive(true); // brake when the commands are zero

            while (opModeIsActive()) {
                follower.update();
                limelight.update();
                Pose pose = follower.getPose();
                double now = run.seconds();

                Limelight.Sighting s = limelight.getPollen(PollenSettings.MIN_AREA, PollenSettings.MAX_STALENESS_MS,
                        haveLast, lastTx, STICKY_DEG);
                if (s != null) {
                    if (seenSince < 0) seenSince = now;
                    lastSeenAt = now;
                    lastTx = s.txDeg;
                    lastDistance = PollenSettings.distanceIn(s.tyDeg);
                    haveLast = true;
                } else {
                    seenSince = -1;
                }
                boolean acquired = s != null && (now - seenSince) >= PollenSettings.ACQUIRE_TIME_SEC;

                double forward = 0, turn = 0;

                if (state == State.SEARCH || state == State.CHASE) {
                    // Safety net, every loop: stop if the robot is somewhere it must never be.
                    String reason = null;
                    if (PollenSettings.ENABLE_FIELD_GUARD) {
                        reason = guard.wallViolation(pose.getX(), pose.getY(), pose.getHeading());
                        if (reason == null) reason = guard.keepOutViolation(pose.getX(), pose.getY(), pose.getHeading());
                    }
                    if (reason == null && Math.hypot(pose.getX() - start.getX(), pose.getY() - start.getY()) > PollenSettings.MAX_TRAVEL_IN) {
                        reason = "moved more than " + PollenSettings.MAX_TRAVEL_IN + " in from the start";
                    }
                    if (reason == null && now > PollenSettings.RUN_TIMEOUT_SEC) {
                        reason = "ran longer than " + PollenSettings.RUN_TIMEOUT_SEC + " s";
                    }
                    if (reason != null) {
                        state = State.ABORTED;
                        note = reason;
                    }
                }

                switch (state) {
                    case SEARCH:
                        if (acquired) {
                            state = State.CHASE;
                            stateTimer.reset();
                        } else if (stateTimer.seconds() > PollenSettings.SEARCH_TIMEOUT_SEC) {
                            state = State.NOT_FOUND;
                            note = "no pollen found in " + PollenSettings.SEARCH_TIMEOUT_SEC + " s";
                        } else {
                            turn = searchDir * searchPower;
                        }
                        break;
                    case CHASE:
                        if (s == null) {
                            if (now - lastSeenAt > PollenSettings.LOST_TIMEOUT_SEC) {
                                if (!Double.isNaN(lastDistance)
                                        && lastDistance <= PollenSettings.STOP_DISTANCE_IN + PollenSettings.NEAR_LOST_MARGIN_IN) {
                                    // It was close and then left the bottom of the picture: the robot has reached it.
                                    state = State.ARRIVED;
                                    note = String.format("pollen went out of view at %.1f in, taken as reached", lastDistance);
                                } else {
                                    // Lost it far away: turn back toward where it was last seen (tx + = right = clockwise).
                                    double txEff = PollenSettings.INVERT_TX ? -lastTx : lastTx;
                                    searchDir = txEff > 0 ? -1 : 1;
                                    haveLast = false;
                                    state = State.SEARCH;
                                    stateTimer.reset();
                                }
                            }
                            // else: hold still for a moment in case it comes back
                        } else {
                            PollenMath.Command c = PollenMath.command(s.txDeg, PollenSettings.distanceIn(s.tyDeg), gains);
                            if (c.arrived) {
                                state = State.ARRIVED;
                                note = String.format("pollen %.1f in away", PollenSettings.distanceIn(s.tyDeg));
                            } else {
                                forward = c.forward;
                                turn = c.turn;
                            }
                        }
                        break;
                    default: // ARRIVED, NOT_FOUND, ABORTED: stand still
                        break;
                }

                follower.setTeleOpDrive(forward, 0, turn, true);

                telemetry.addData("state", note.isEmpty() ? state : state + ": " + note);
                telemetry.addData("pollen", s == null ? "none" : String.format("tx %.1f  ty %.1f  area %.3f%%  dist %.1f in",
                        s.txDeg, s.tyDeg, s.area * 100, PollenSettings.distanceIn(s.tyDeg)));
                telemetry.addData("command", "forward %.2f  turn %.2f", forward, turn);
                telemetry.addData("pose", "(%.1f, %.1f) %.0f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
                telemetry.addData("Limelight", limelight.statusLine());
                telemetry.update();
                Drawing.drawDebug(follower);
            }
        } finally {
            follower.breakFollowing();   // motors off
            follower.setMaxPower(1.0);   // the follower settings are shared with the other OpModes
            limelight.stop();
        }
    }
}
