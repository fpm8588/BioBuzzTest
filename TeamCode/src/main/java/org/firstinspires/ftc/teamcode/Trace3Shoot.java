package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * TRACE 3 + SHOOT: shoots first, then drives the Trace 3 route.
 *
 *   1. START: the shoot step (ShootSequence): the shooter spins up to Shoot.SHOOT_RPM (2100), then the intake runs for ShootSequence.INTAKE_SEC
 *      (6 s) once SPIN_UP_MIN_SEC (3 s) have passed and the shooter is up to speed, then both are switched off. The robot stays where it is.
 *   2. The robot drives the Trace 3 route exactly as Trace 3 does (same poses, same checks, same safety stops, red / blue switch).
 *
 * The route is Trace 3's: this class extends Trace3, so a change to Trace 3 applies here too. The shoot settings (INTAKE_SEC, SPIN_UP_MIN_SEC,
 * SPIN_UP_MAX_SEC) are in Panels under ShootSequence. The checks, refusal rules and the other settings are in RouteAutoBase and LeaveAndPark
 * (Panels > LeaveAndPark). The route timeout starts after the shoot step.
 *
 * Needs the same hardware as the TeleOp: motors "shoot" and "intake" on the Expansion Hub with their encoders (docs/HARDWARE_CONFIG.md).
 * The two motors are switched off when the OpMode ends for any reason, including STOP.
 *
 * Time: the shoot step takes up to SPIN_UP_MAX_SEC + INTAKE_SEC = 11 s of the 30 s autonomous, then the route.
 */
@Autonomous(name = "Trace 3 Shoot", group = "Competition")
public class Trace3Shoot extends Trace3 {

    private ShootSequence shootSequence;

    @Override
    protected String title() {
        return "TRACE 3 + SHOOT";
    }

    @Override
    protected void onInit() {
        shootSequence = new ShootSequence(this);
    }

    @Override
    protected void initTelemetry() {
        shootSequence.initTelemetry();
    }

    @Override
    protected void beforeRoute() {
        shootSequence.run();
    }

    @Override
    protected void onFinish() {
        shootSequence.stop();
    }
}
