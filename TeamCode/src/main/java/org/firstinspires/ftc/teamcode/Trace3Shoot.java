package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shoot;

/*
 * TRACE 3 + SHOOT: shoots first, then drives the Trace 3 route.
 *
 *   1. START: the shooter spins up to Shoot.SHOOT_RPM (2100). The robot stays where it is.
 *   2. Once the shooter is up to speed (or after SPIN_UP_MAX_SEC, whichever is first) the intake runs in at Intake.INTAKE_RPM (900) for
 *      INTAKE_SEC (6 s), feeding the game pieces to the shooter, which keeps spinning.
 *   3. Both are switched off.
 *   4. The robot drives the Trace 3 route exactly as Trace 3 does (same poses, same checks, same safety stops, red / blue switch).
 *
 * The route is Trace 3's: this class extends Trace3, so a change to Trace 3 applies here too. The checks, refusal rules and settings are in
 * RouteAutoBase and LeaveAndPark (Panels > LeaveAndPark). The route timeout starts after step 3.
 *
 * Needs the same hardware as the TeleOp: motors "shoot" and "intake" on the Expansion Hub with their encoders (docs/HARDWARE_CONFIG.md).
 * The two motors are switched off when the OpMode ends for any reason, including STOP.
 *
 * Time: step 1 to 3 take up to SPIN_UP_MAX_SEC + INTAKE_SEC = 9 s of the 30 s autonomous, then the route.
 */
@Configurable
@Autonomous(name = "Trace 3 Shoot", group = "Competition")
public class Trace3Shoot extends Trace3 {

    /** How long the intake runs, seconds. */
    public static double INTAKE_SEC = 6.0;

    /**
     * Longest wait for the shooter to reach speed before the intake starts anyway, seconds. 0 = do not wait: the intake starts at once.
     * With the shooter at 2100 RPM it should be up to speed in about a second.
     */
    public static double SPIN_UP_MAX_SEC = 3.0;

    private Shoot shoot;
    private Intake intake;

    @Override
    protected String title() {
        return "TRACE 3 + SHOOT";
    }

    @Override
    protected void onInit() {
        shoot = new Shoot(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    protected void initTelemetry() {
        telemetry.addLine(String.format("First: shooter at %.0f RPM, then intake at %.0f RPM for %.1f s, both off, then the Trace 3 route.",
                Shoot.SHOOT_RPM, Intake.INTAKE_RPM, INTAKE_SEC));
        if (shoot.problem() != null) telemetry.addData("SHOOT PROBLEM", shoot.problem());
        if (intake.problem() != null) telemetry.addData("INTAKE PROBLEM", intake.problem());
    }

    @Override
    protected void beforeRoute() {
        ElapsedTime timer = new ElapsedTime();

        // 1. Shooter on, wait for it to reach speed.
        shoot.spin();
        while (opModeIsActive() && timer.seconds() < SPIN_UP_MAX_SEC && !shoot.isAtSpeed()) {
            telemetry.addData("state", "spinning up the shooter");
            telemetry.addData("shoot RPM", "%.0f (target %.0f)", shoot.getRpm(), Shoot.SHOOT_RPM);
            telemetry.update();
            sleep(20);
        }

        if (!opModeIsActive()) return; // STOP was pressed: onFinish() switches the shooter off

        // 2. Intake on for INTAKE_SEC, shooter still spinning.
        timer.reset();
        intake.intakeIn();
        while (opModeIsActive() && timer.seconds() < INTAKE_SEC) {
            telemetry.addData("state", "intake running: %.1f s left", Math.max(0, INTAKE_SEC - timer.seconds()));
            telemetry.addData("shoot RPM", "%.0f (target %.0f)", shoot.getRpm(), Shoot.SHOOT_RPM);
            telemetry.addData("intake RPM", "%.0f (target %.0f)", intake.getRpm(), Intake.INTAKE_RPM);
            telemetry.update();
            sleep(20);
        }

        // 3. Both off, then the route starts.
        shoot.stop();
        intake.stop();
    }

    @Override
    protected void onFinish() {
        shoot.stop();
        intake.stop();
    }
}
