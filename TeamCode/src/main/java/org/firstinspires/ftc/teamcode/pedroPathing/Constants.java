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
            // TODO(wiring): motor-config name of the port each pod's encoder is plugged into. Use ports whose
            // motor does not need its own encoder (drive motors, intake), never the shooter.
            .forwardEncoder_HardwareMapName("lf")
            .strafeEncoder_HardwareMapName("rb")
            .IMU_HardwareMapName("imu")
            // TODO: must match how the Control Hub is mounted on the robot.
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT))
            // Offsets are from the tracking point O (frame centreline x midway between the axle lines), in inches.
            // forwardPodY: sideways offset of the FORWARD pod wheel, LEFT of centre positive.
            // strafePodX:  fore/aft offset of the STRAFE pod wheel, FORWARD of centre positive.
            // PLANNED values (pods near the axes through O): tape-measure and replace once mounted.
            .forwardPodY(0.0)
            .strafePodX(0.0)
            // TODO: inches per encoder tick. Estimates from 2000 ticks/rev: 4-bar pod (32 mm wheel) 0.001979,
            // swingarm pod (48 mm wheel) 0.002968. Refine with Pedro's Forward/Lateral tuners.
            .forwardTicksToInches(0.001979)
            .strafeTicksToInches(0.001979)
            // TODO: flip a direction if its pod counts down when the robot moves forward (forward pod)
            // or left (strafe pod).
            .forwardEncoderDirection(Encoder.FORWARD)
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
