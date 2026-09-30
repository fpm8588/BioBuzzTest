// @ftc-toolchain generated: pedro-constants — scaffolded; tune every value
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Pedro Pathing constants. EVERY value here is robot-specific:
 * run the tuning OpModes and follow https://pedropathing.com/docs/pathing/tuning
 * before trusting any path. This scaffold assumes a mecanum drivetrain with a
 * goBILDA Pinpoint localizer; swap the localizer/drivetrain builder calls if
 * your robot differs (see docs: pathing/tuning/localization).
 */
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.0) // TODO: robot mass in kg
            .forwardZeroPowerAcceleration(-34.0) // TODO: from Forward Zero Power Acceleration tuner
            .lateralZeroPowerAcceleration(-78.0) // TODO: from Lateral Zero Power Acceleration tuner
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.05, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01))
            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("lf")   // matches team's existing Driver Station config names
            .leftRearMotorName("lb")
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(57.8)  // TODO: from Forward Velocity tuner
            .yVelocity(52.3); // TODO: from Strafe Velocity tuner

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.0)  // TODO: measure once pods are mounted (see pod offset guidance)
            .strafePodX(-2.5)
            .hardwareMapName("pinpoint") // I2C port 1, 2, or 3 on the Driver Station config — never port 0
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, 0.1, 0.1, 0.009, 50, 1.25, 10, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
