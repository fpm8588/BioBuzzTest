package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shoot;

/**
 * The shoot step used by the autos that shoot first (Trace3Shoot, Trace4). The robot stands still while:
 *   1. the shooter spins up to Shoot.SHOOT_RPM;
 *   2. the intake starts when at least SPIN_UP_MIN_SEC have passed since the shooter started AND the shooter is up to speed (or
 *      SPIN_UP_MAX_SEC has passed, so a shooter that never gets there does not hold the auto up for ever);
 *   3. the intake runs in at Intake.INTAKE_RPM for INTAKE_SEC, feeding the game pieces to the shooter, which keeps spinning;
 *   4. both are switched off.
 *
 * Create it at INIT (a missing motor then fails at INIT, not at START), call run() when it is time, and stop() when the OpMode ends.
 * The three settings are in Panels (class ShootSequence) and apply to every auto that uses it.
 */
@Configurable
public class ShootSequence {

    /** How long the intake runs, seconds. */
    public static double INTAKE_SEC = 6.0;

    /** The intake never starts sooner than this after the shooter starts, even if the shooter is already up to speed, seconds. Raise it for a longer wait. */
    public static double SPIN_UP_MIN_SEC = 3.0;

    /**
     * Longest wait for the shooter to reach speed before the intake starts anyway, seconds (counted from the shooter starting; if it is
     * less than SPIN_UP_MIN_SEC the minimum wins). Set both to 0 to start the intake at once.
     */
    public static double SPIN_UP_MAX_SEC = 5.0;

    private final LinearOpMode opMode;
    private final Shoot shoot;
    private final Intake intake;

    public ShootSequence(LinearOpMode opMode) {
        this.opMode = opMode;
        shoot = new Shoot(opMode.hardwareMap);
        intake = new Intake(opMode.hardwareMap);
    }

    /** Lines for the INIT screen: what it will do, and any problem with the two motors' set-up. */
    public void initTelemetry() {
        opMode.telemetry.addLine(String.format("Shoot: shooter at %.0f RPM, %.1f s later the intake at %.0f RPM for %.1f s, then both off.",
                Shoot.SHOOT_RPM, SPIN_UP_MIN_SEC, Intake.INTAKE_RPM, INTAKE_SEC));
        if (shoot.problem() != null) opMode.telemetry.addData("SHOOT PROBLEM", shoot.problem());
        if (intake.problem() != null) opMode.telemetry.addData("INTAKE PROBLEM", intake.problem());
    }

    /** Do the whole step. Returns when it is done, or at once if STOP is pressed (call stop() afterwards in any case). */
    public void run() {
        ElapsedTime timer = new ElapsedTime();

        // 1. Shooter on. Wait at least SPIN_UP_MIN_SEC, and then until it is up to speed (but no longer than SPIN_UP_MAX_SEC).
        shoot.spin();
        while (opMode.opModeIsActive() && (timer.seconds() < SPIN_UP_MIN_SEC || (timer.seconds() < SPIN_UP_MAX_SEC && !shoot.isAtSpeed()))) {
            opMode.telemetry.addData("state", "spinning up the shooter: %.1f s", timer.seconds());
            opMode.telemetry.addData("shoot RPM", "%.0f (target %.0f)", shoot.getRpm(), Shoot.SHOOT_RPM);
            opMode.telemetry.update();
            opMode.sleep(20);
        }
        if (!opMode.opModeIsActive()) return; // STOP was pressed: stop() switches the shooter off

        // 2. Intake on for INTAKE_SEC, shooter still spinning.
        timer.reset();
        intake.intakeIn();
        while (opMode.opModeIsActive() && timer.seconds() < INTAKE_SEC) {
            opMode.telemetry.addData("state", "intake running: %.1f s left", Math.max(0, INTAKE_SEC - timer.seconds()));
            opMode.telemetry.addData("shoot RPM", "%.0f (target %.0f)", shoot.getRpm(), Shoot.SHOOT_RPM);
            opMode.telemetry.addData("intake RPM", "%.0f (target %.0f)", intake.getRpm(), Intake.INTAKE_RPM);
            opMode.telemetry.update();
            opMode.sleep(20);
        }

        // 3. Both off.
        stop();
    }

    /** Switch the shooter and the intake off. Safe to call any time, any number of times. */
    public void stop() {
        shoot.stop();
        intake.stop();
    }
}
