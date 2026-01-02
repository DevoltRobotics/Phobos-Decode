package org.firstinspires.ftc.teamcode.PoseEstimate;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Rotation2d;

public class ConversionUtil {

    public static Pose2d pedroToWPI(Pose pose) {
        return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getHeading()));
    }

    public static Pose WPIToPedro(Pose2d pose){

        Pose2D inFTCPose = new Pose2D(DistanceUnit.METER, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getRotation().getRadians());

        Pose ftcStandard = PoseConverter.pose2DToPose(inFTCPose, InvertedFTCCoordinates.INSTANCE);

        return ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    public static Pose2d from3DToPedro(Pose3D pose3d, double heading){
        Pose2D convertedPose = new Pose2D(
                DistanceUnit.METER,
                pose3d.getPosition().x,
                pose3d.getPosition().y,
                AngleUnit.RADIANS,
                heading
        );

        Pose ftcStandard = PoseConverter.pose2DToPose(convertedPose, InvertedFTCCoordinates.INSTANCE);
        Pose pedroStandard = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        return
                new Pose2d(
                        pedroStandard.getX(),
                        pedroStandard.getY(),
                        Rotation2d.fromRadians(heading)
                );
    }
}
