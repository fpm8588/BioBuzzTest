// @ftc-toolchain generated: pedro-constants — scaffolded; tune every value
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Pedro Pathing constants. EVERY value here is robot-specific:
 * run the tuning OpModes and follow https://pedropathing.com/docs/pathing/tuning
 * before trusting any path. This scaffold assumes a mecanum drivetrain with a
 * two-dead-wheel + IMU localizer; swap the localizer/drivetrain builder calls if
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

    // Two goBILDA dead wheels + the Control Hub IMU (no Pinpoint). Pod encoders plug into REV hub encoder
    // ports; heading comes from the hub IMU. See docs/ODOMETRY.md for placement, wiring and verification.
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            // Each pod's encoder plugs into the encoder connector of the Control Hub port of this motor
            // (see docs/HARDWARE_CONFIG.md). rf/rb are drive motors that don't need their own encoders.
            .forwardEncoder_HardwareMapName("rf")
            .strafeEncoder_HardwareMapName("rb")
            .IMU_HardwareMapName("imu")
            // Control Hub mounted with the REV logo facing the robot's front and the USB port facing up.
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP))
            // Offsets in inches from the robot's center of rotation.
            // forwardPodY: sideways offset of the FORWARD pod wheel, LEFT of centre positive.
            // strafePodX:  fore/aft offset of the STRAFE pod wheel, FORWARD of centre positive.
            // From the Offsets Tuner, average of 3 runs: forwardY 0.4168, 0.1921, -0.0543; strafeX 0.1711, 0.2964, 0.1304.
            // Re-run the Offsets Tuner after the other subsystems are mounted (the center of rotation moves).
            .forwardPodY(0.185)
            .strafePodX(0.199)
            // Inches per encoder tick. Forward: Forward Tuner, average of 3 runs (0.0019672, 0.0019724, 0.0019688).
            // Strafe: Lateral Tuner, average of 3 runs (0.0019752, 0.0019754, 0.0019753; the tuner showed them negative).
            .forwardTicksToInches(0.0019695)
            .strafeTicksToInches(0.0019753)
            // Forward pod flipped (x was reversed in the Localization Test). Strafe pod is not flipped: the Lateral Tuner read
            // negative only because the robot was pushed right. TODO: confirm in the Localization Test (push left: y must rise).
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, 0.1, 0.1, 0.009, 50, 1.25, 10, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
