package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer.EKFLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer.EstimatorLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer.KalmanPedroLocalizer;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-32.46)
            .lateralZeroPowerAcceleration(-55.04)

            .headingPIDFCoefficients(new PIDFCoefficients(0.095,0,0.05,0.025))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.45, 0, 0.05, 0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0,0.008, 0.6, 0.12))

            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5,0,0.13,0.025))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.18, 0, 0.035, 0.04))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0,0.01, 0.6, 0.098))

            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryDrivePIDF(false)

            .drivePIDFSwitch(10)
            .translationalPIDFSwitch(1.5)

            .centripetalScaling(0.00052) //0.00052

            //.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.05, 0.0485, 0.00262))
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .useBrakeModeInTeleOp(true)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.9)
            .yVelocity(59.5)

            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            //.forwardPodY(-1.85) //-2
            //.strafePodX(-5) //-4

            .forwardPodY(-2) //-2
            .strafePodX(-4) //-4

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
            2.7,
            10,
            1.5
    );

    public static Follower createFollower(HardwareMap hardwareMap, Limelight3A limelight) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                //.pinpointLocalizer(localizerConstants)
                .setLocalizer(new KalmanPedroLocalizer(new PinpointLocalizer(hardwareMap, localizerConstants), limelight))
                .build();

    }
}
