package org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer;

import static org.firstinspires.ftc.teamcode.PoseEstimate.ConversionUtil.WPIToPedro;
import static org.firstinspires.ftc.teamcode.PoseEstimate.ConversionUtil.pedroToWPI;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.Units;

public class KalmanPedroLocalizer implements Localizer{

    private final PinpointLocalizer pinpointLocalizer;
    private final Limelight3A limelight;

    private Pose estimatedPose;

    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0;

    // Covariance values (tune these)
    private double varX = 1.0;
    private double varY = 1.0;
    private double varHeading = 0.5;

    private double lastHeading = 0;

    // Noise parameters
    private final double processNoise = 0.02;
    private final double visionNoise = 0.4;

    private static final double ANGULAR_VEL_THRESHOLD = Math.toRadians(180);

    public KalmanPedroLocalizer(PinpointLocalizer pinpointLocalizer, Limelight3A ll) {
        this.pinpointLocalizer = pinpointLocalizer;
        this.limelight = ll;

        estimatedPose = pinpointLocalizer.getPose();
    }

    @Override
    public Pose getVelocity() {
        return pinpointLocalizer.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return pinpointLocalizer.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        setPose(setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        pinpointLocalizer.setPose(setPose);

    }

    @Override
    public void update() {

        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // ----------------------------
        // PREDICT STEP (Pinpoint Odometry)
        // ----------------------------
        pinpointLocalizer.update();
        Pose odoPose = pinpointLocalizer.getPose();

        double heading = odoPose.getHeading(); // Always trust this

        estimatedPose = new Pose(
                odoPose.getX(),
                odoPose.getY(),
                heading
        );

        // Increase positional uncertainty over time
        varX += processNoise * dt;
        varY += processNoise * dt;

        // ----------------------------
        // ANGULAR VELOCITY CHECK
        // ----------------------------
        double angularVelocity = (heading - lastHeading) / dt;
        lastHeading = heading;

        if (Math.abs(angularVelocity) > Math.toRadians(180)) {
            return; // Reject vision if turning too fast
        }

        // ----------------------------
        // VISION UPDATE (MT2 X/Y ONLY)
        // ----------------------------
        limelight.updateRobotOrientation(heading);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double visionX = result.getBotpose_MT2().getPosition().x;
            double visionY = result.getBotpose_MT2().getPosition().y;

            // Kalman gain for X/Y
            double kX = varX / (varX + visionNoise);
            double kY = varY / (varY + visionNoise);

            double fusedX = estimatedPose.getX()
                    + kX * (visionX - estimatedPose.getX());

            double fusedY = estimatedPose.getY()
                    + kY * (visionY - estimatedPose.getY());

            estimatedPose = new Pose(
                    fusedX,
                    fusedY,
                    heading // Never modify heading
            );

            // Reduce covariance
            varX *= (1 - kX);
            varY *= (1 - kY);
        }
    }

    @Override
    public double getTotalHeading() {
        return pinpointLocalizer.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return pinpointLocalizer.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return pinpointLocalizer.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return pinpointLocalizer.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {

    }

    @Override
    public double getIMUHeading() {
        return pinpointLocalizer.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return pinpointLocalizer.isNAN();
    }

    public Pose getPose() {
        return estimatedPose;
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}