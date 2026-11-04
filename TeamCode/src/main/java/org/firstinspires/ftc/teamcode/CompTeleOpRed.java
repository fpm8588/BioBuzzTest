package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Competition TeleOp for the RED alliance. Identical to the BLUE one except that the aim button (driver right bumper)
 * turns to face the red scoring strip of AprilTags (IDs 30 to 33). All the behaviour is in CompTeleOpBase.
 */
@TeleOp(name = "Competition TeleOp RED", group = "Competition")
public class CompTeleOpRed extends CompTeleOpBase {

    @Override
    protected String allianceName() {
        return "RED";
    }

    @Override
    protected int stripFirstTagId() {
        return AprilTagSettings.RED_STRIP_FIRST_ID;
    }
}
