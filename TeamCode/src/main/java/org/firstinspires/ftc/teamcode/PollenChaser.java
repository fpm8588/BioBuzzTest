package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.LeaveParkPlan;
import org.firstinspires.ftc.teamcode.util.PollenMath;

/*
 * The pollen chase as a part an OpMode can start whenever it likes (DRIVEANDGRAB starts it after a route). It is the same
 * behaviour and the same numbers (PollenSettings) as the Pollen Chase OpMode:
 *
 *   SEARCH (spin until a pollen has been seen for ACQUIRE_TIME_SEC) -> CHASE (turn and drive up to it) -> ARRIVED.
 *   Lost far away: turn back toward where it was and search again. Lost close in: counts as reached.
 *
 * The caller owns the loop. Every loop: follower.update(), then update(now). The chaser takes over the drivetrain in
 * teleop mode while it runs, so call finish() when done to put the follower back (motors off, full max power).
 *
 * Safety, every loop: the field guard (walls, HIVE, FLOWERS; ENABLE_FIELD_GUARD), MAX_TRAVEL_IN from where the chase
 * began, and RUN_TIMEOUT_SEC counted from begin(). If any trips, the motors stop and the state is ABORTED.
 */
public final class PollenChaser {

    public enum State { SEARCH, CHASE, ARRIVED, NOT_FOUND, ABORTED }

    private static final double STICKY_DEG = 5.0; // keep following the same ball if it moves less than this between frames

    private final Follower follower;
    private final Limelight limelight;
    private final LeaveParkPlan guard;
    private final PollenMath.Gains gains = PollenSettings.gains();

    private State state = State.SEARCH;
    private String note = "";
    private Pose origin;
    private double beganAt, stateSince;
    private double searchDir = 1, searchPower;
    private boolean haveLast;
    private double lastTx, seenSince = -1, lastSeenAt = -1, lastDistance = Double.NaN, prevForward;

    private Limelight.Sighting sighting;
    private double forward, turn;

    public PollenChaser(Follower follower, Limelight limelight, LeaveParkPlan guard) {
        this.follower = follower;
        this.limelight = limelight;
        this.guard = guard;
    }

    /** Starts the chase from the robot's current pose. now = seconds on the caller's clock. */
    public void begin(Pose origin, double now) {
        begin(origin, now, 0);
    }

    /**
     * Same, but searching in a chosen direction first: +1 = counter-clockwise (left), -1 = clockwise (right), 0 = the
     * direction of PollenSettings.SEARCH_TURN_POWER. Use it to look toward where pollen is likely to be.
     */
    public void begin(Pose origin, double now, int firstSearchDirection) {
        this.origin = origin;
        beganAt = now;
        stateSince = now;
        state = State.SEARCH;
        note = "";
        searchPower = Math.min(0.5, Math.abs(PollenSettings.SEARCH_TURN_POWER));
        searchDir = firstSearchDirection != 0 ? Math.signum(firstSearchDirection) : (PollenSettings.SEARCH_TURN_POWER >= 0 ? 1 : -1);
        haveLast = false;
        seenSince = -1;
        lastSeenAt = -1;
        lastDistance = Double.NaN;
        prevForward = 0;
        follower.setMaxPower(Math.max(0.1, Math.min(1.0, PollenSettings.CHASE_MAX_POWER)));
        follower.startTeleopDrive(true); // takes over from any path; brakes when the commands are zero
    }

    /** One step. Call after follower.update(). */
    public void update(double now) {
        limelight.update();
        Pose pose = follower.getPose();

        sighting = limelight.getPollen(PollenSettings.MIN_AREA, PollenSettings.MAX_STALENESS_MS, haveLast, lastTx, STICKY_DEG);
        if (sighting != null) {
            if (seenSince < 0) seenSince = now;
            lastSeenAt = now;
            lastTx = sighting.txDeg;
            lastDistance = PollenSettings.distanceIn(sighting.tyDeg);
            haveLast = true;
        } else {
            seenSince = -1;
        }
        boolean acquired = sighting != null && (now - seenSince) >= PollenSettings.ACQUIRE_TIME_SEC;

        forward = 0;
        turn = 0;

        if (state == State.SEARCH || state == State.CHASE) {
            // Safety net, every loop: stop if the robot is somewhere it must never be.
            String reason = null;
            if (PollenSettings.ENABLE_FIELD_GUARD) {
                reason = guard.wallViolation(pose.getX(), pose.getY(), pose.getHeading());
                if (reason == null) reason = guard.keepOutViolation(pose.getX(), pose.getY(), pose.getHeading());
            }
            if (reason == null && Math.hypot(pose.getX() - origin.getX(), pose.getY() - origin.getY()) > PollenSettings.MAX_TRAVEL_IN) {
                reason = "moved more than " + PollenSettings.MAX_TRAVEL_IN + " in from where the chase began";
            }
            if (reason == null && now - beganAt > PollenSettings.RUN_TIMEOUT_SEC) {
                reason = "chase ran longer than " + PollenSettings.RUN_TIMEOUT_SEC + " s";
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
                    stateSince = now;
                } else if (now - stateSince > PollenSettings.SEARCH_TIMEOUT_SEC) {
                    state = State.NOT_FOUND;
                    note = "no pollen found in " + PollenSettings.SEARCH_TIMEOUT_SEC + " s";
                } else {
                    turn = searchDir * searchPower;
                }
                break;
            case CHASE:
                if (sighting == null) {
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
                            stateSince = now;
                        }
                    }
                    // else: hold still for a moment in case it comes back
                } else {
                    double dist = PollenSettings.distanceIn(sighting.tyDeg);
                    PollenMath.Command c = PollenMath.command(sighting.txDeg, dist, gains, prevForward);
                    if (c.arrived) {
                        state = State.ARRIVED;
                        note = String.format("pollen %.1f in away", dist);
                    } else {
                        forward = c.forward;
                        turn = c.turn;
                    }
                }
                break;
            default: // ARRIVED, NOT_FOUND, ABORTED: stand still
                break;
        }

        prevForward = forward;
        follower.setTeleOpDrive(forward, 0, turn, true);
    }

    /** Motors off and the follower's max power back to full. Safe to call any time, even if begin() never ran. */
    public void finish() {
        follower.breakFollowing();
        follower.setMaxPower(1.0);
    }

    public State getState() { return state; }

    public String getNote() { return note; }

    /** The pollen being followed this loop, or null. */
    public Limelight.Sighting getSighting() { return sighting; }

    public double getForward() { return forward; }

    public double getTurn() { return turn; }

    /** True once the chase has reached a final state (ARRIVED, NOT_FOUND or ABORTED). */
    public boolean isDone() { return state != State.SEARCH && state != State.CHASE; }
}
