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

    public static Pose WPIToPedro(Pose2d pose) {

        return new Pose(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }

    public static Pose2d from3DToPedro(Pose3D pose3d, double headingRad) {
        double xFtc = pose3d.getPosition().x * 39.37;
        double yFtc = pose3d.getPosition().y * 39.37;
        // 1) shift origin: center → corner
        double xShifted = 72 - xFtc;
        double yShifted = yFtc + 72.0;

        // 2) axis swap
        double xPedro = yShifted;
        double yPedro = xShifted;

        // 3) heading conversion
        double headingPedro = -headingRad;

        return new Pose2d(
                xPedro,
                yPedro,
                Rotation2d.fromRadians(headingPedro)
        );
    }
    /*
    public static Pose2d from3DToPedro(Pose3D pose3d, double heading){
        Pose2D convertedPose = new Pose2D(
                DistanceUnit.INCH,
                pose3d.getPosition().x * 39.37,
                pose3d.getPosition().y * 39.37,
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

     */
}
