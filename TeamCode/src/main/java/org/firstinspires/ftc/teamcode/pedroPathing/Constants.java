package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-33.4)
            .lateralZeroPowerAcceleration(-73.1)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.037, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7,0,0.03,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.9, 0,0.05, 0.3, 0))

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.02, 0.015))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.75,0,0.03,0.015))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0,0 , 0.6, 0))

            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)

            .centripetalScaling(0.00052)

            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.85)
            .useBrakeModeInTeleOp(true)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75)
            .yVelocity(57.4)



            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1)
            .strafePodX(-1.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")

            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009, //0.009
                150,
            2,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder( followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }


}
