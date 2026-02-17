package org.firstinspires.ftc.teamcode.PoseEstimate;

import static org.firstinspires.ftc.teamcode.PoseEstimate.ConversionUtil.*;
import static org.firstinspires.ftc.teamcode.PoseEstimate.ConversionUtil.pedroToWPI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PoseEstimate.pinpoint.PinpointKinematics;
import org.firstinspires.ftc.teamcode.PoseEstimate.pinpoint.PinpointOdometry;
import org.firstinspires.ftc.teamcode.PoseEstimate.pinpoint.PinpointPoseEstimator;
import org.firstinspires.ftc.teamcode.PoseEstimate.pinpoint.PinpointPositions;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.Units;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.VecBuilder;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Rotation2d;

public class VisionPinpointLocalizer implements Localizer {

    PinpointKinematics kinematics = new PinpointKinematics();
    PinpointOdometry odometry = new PinpointOdometry(kinematics, Rotation2d.kZero, new PinpointPositions(), Pose2d.kZero);
    PinpointPoseEstimator estimator = new PinpointPoseEstimator(odometry,
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(.25, .25, 9999999)
    );

    Limelight3A ll;
    PinpointLocalizer localizer;

    public VisionPinpointLocalizer(Limelight3A ll, PinpointLocalizer localizer) {
        this.localizer = localizer;

        this.ll = ll;
    }

    @Override
    public Pose getPose() {
        return WPIToPedro(estimator.getEstimatedPosition());
    }

    @Override
    public Pose getVelocity() {
        return localizer.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return localizer.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        setPose(setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        estimator.resetPose(pedroToWPI(setPose));

    }

    @Override
    public void update() {
        localizer.update();

        PinpointPositions positions = new PinpointPositions(
                localizer.getPose().getX(),
                localizer.getPose().getY(),
                localizer.getPose().getHeading()
        );

        estimator.update(Rotation2d.fromRadians(localizer.getPose().getHeading()), positions);

        ll.updateRobotOrientation(Units.radiansToDegrees(localizer.getPose().getHeading()) + 90);

        LLResult result = ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();

                //Pose2d mtPose = from3DToPedro(botpose, localizer.getPose().getHeading());

                estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.25, .25, 9999999));
                //estimator.addVisionMeasurement(mtPose, result.getTimestamp());

                FtcDashboard.getInstance().getTelemetry().addData("botX", botpose.getPosition().x);
                FtcDashboard.getInstance().getTelemetry().addData("botY", botpose.getPosition().y);
                FtcDashboard.getInstance().getTelemetry().addData("botHeading", botpose.getOrientation());

                FtcDashboard.getInstance().getTelemetry().addData("mtPoseX", estimator.getEstimatedPosition().getX());
                FtcDashboard.getInstance().getTelemetry().addData("mtPoseY", estimator.getEstimatedPosition().getY());
                FtcDashboard.getInstance().getTelemetry().addData("mtPoseHeading", estimator.getEstimatedPosition().getRotation());

            }
        }

        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public double getTotalHeading() {
        return localizer.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return localizer.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return localizer.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return localizer.getTurningMultiplier();
    }

    @Override
    public void resetIMU() {
        localizer.resetIMU();
        odometry.resetRotation(new Rotation2d());
    }

    @Override
    public double getIMUHeading() {
        return localizer.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return localizer.isNAN();
    }
}
