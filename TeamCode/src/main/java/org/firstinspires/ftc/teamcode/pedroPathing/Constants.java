package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstantsTeleOp = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-31.66)
            .lateralZeroPowerAcceleration(-73.14)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0.00002, 0.00002, 0.033))
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0.05, 0.01));

    public static FollowerConstants followerConstantsAuton = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-31.66)
            .lateralZeroPowerAcceleration(-73.14)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0.00002, 0.00002, 0.033))
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0.05, 0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(64.622)
            .yVelocity(31.65);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-118.033)
            .strafePodX(39.999)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstantsTeleOp, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }

    public static Follower createAngel(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstantsAuton,hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }


}