package org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer;

import static org.firstinspires.ftc.teamcode.PoseEstimate.ConversionUtil.from3DToPedro;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class EstimatorLocalizer implements Localizer {

    private final PinpointLocalizer pinpoint;
    private final Limelight3A limelight;
    private final PinpointPoseEstimator estimator;

    private Pose currentPose;
    private double lastHeading;
    private double lastTime;

    private final ElapsedTime timer = new ElapsedTime();


    private final double MAX_ANGULAR_VELOCITY =
            Math.toRadians(180);

    public EstimatorLocalizer(
            PinpointLocalizer pinpoint,
            Limelight3A limelight

    ) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;

        this.currentPose = pinpoint.getPose();

        estimator = new PinpointPoseEstimator(
                currentPose,
                0.02, 0.02,
                0.25, 0.25
        );

        timer.reset();
    }

    // ------------------------------------------------
    // CALL IN LOOP
    // ------------------------------------------------

    @Override
    public void update() {

        pinpoint.update();
        Pose odometryPose = pinpoint.getPose();

        double currentTime = timer.seconds();

        double dt = currentTime - lastTime;
        double heading = odometryPose.getHeading();

        double angularVelocity =
                (heading - lastHeading) / Math.max(dt, 1e-6);

        lastHeading = heading;
        lastTime = currentTime;

        // Handle vision
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            if (Math.abs(angularVelocity) < MAX_ANGULAR_VELOCITY) {

                double latency =
                        result.getCaptureLatency() / 1000.0;

                double timestamp =
                        currentTime - latency;

                Pose3D botpose = result.getBotpose();

                Pose visionPose = from3DToPedro(botpose, heading);

                estimator.addVisionMeasurement(
                        visionPose,
                        timestamp
                );
            }
        }

        currentPose = estimator.update(odometryPose);
    }

    @Override
    public Pose getPose() {
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        return pinpoint.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return pinpoint.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        setPose(setStart);
    }

    @Override
    public void setPose(Pose pose) {

        currentPose = pose;

        estimator.addVisionMeasurement(pose, 0);

        pinpoint.setPose(pose);
    }

    @Override
    public double getTotalHeading() {
        return pinpoint.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return pinpoint.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return pinpoint.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return pinpoint.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {

    }

    @Override
    public double getIMUHeading() {
        return pinpoint.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return pinpoint.isNAN();
    }

}
