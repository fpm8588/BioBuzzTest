package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Competition TeleOp for the BLUE alliance. Identical to the RED one except that the aim button (driver right bumper)
 * turns to face the blue scoring strip of AprilTags (IDs 42 to 45). All the behaviour is in CompTeleOpBase.
 */
@TeleOp(name = "Competition TeleOp BLUE", group = "Competition")
public class CompTeleOpBlue extends CompTeleOpBase {

    @Override
    protected String allianceName() {
        return "BLUE";
    }

    @Override
    protected int stripFirstTagId() {
        return AprilTagSettings.BLUE_STRIP_FIRST_ID;
    }
}
