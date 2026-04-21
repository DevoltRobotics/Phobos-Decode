package org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer;

import static org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer.ConversionUtil.from3DToPedro;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
public class KalmanPedroLocalizer implements Localizer {

    private final PinpointLocalizer pinpointLocalizer;
    private final Limelight3A limelight;

    private Pose estimatedPose;
    private Pose lastOdoPose;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    // Covariance values
    private double varX = 0.2;
    private double varY = 0.2;

    private double lastHeading = 0;

    // Noise parameters (tune these)
    private final double processNoise = 0.02;
    private final double visionNoise = 0.4;

    private static final double ANGULAR_VEL_THRESHOLD =
            Math.toRadians(180);

    public KalmanPedroLocalizer(
            PinpointLocalizer pinpointLocalizer,
            Limelight3A ll
    ) {
        this.pinpointLocalizer = pinpointLocalizer;
        this.limelight = ll;
        //this.imu = imu;

        //pinpointLocalizer.getPinpoint().resetPosAndIMU();
        pinpointLocalizer.getPinpoint().recalibrateIMU();

        estimatedPose = pinpointLocalizer.getPose();
        lastOdoPose = estimatedPose;

        timer.reset();
    }

    // ------------------------------------------------
    // PEDRO REQUIRED METHODS
    // ------------------------------------------------

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
        pinpointLocalizer.setStartPose(setStart);
        estimatedPose = setStart;
        lastOdoPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        pinpointLocalizer.setPose(setPose);
        estimatedPose = setPose;
        lastOdoPose = setPose;
    }

    // ------------------------------------------------
    // MAIN UPDATE LOOP
    // ------------------------------------------------

    @Override
    public void update() {

        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt <= 0) dt = 1e-6;

        // ----------------------------
        // PREDICT STEP (DELTA ODOMETRY)
        // ----------------------------

        pinpointLocalizer.update();
        Pose odoPose = pinpointLocalizer.getPose();

        double heading = odoPose.getHeading();

        //double heading = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        // Compute odometry delta
        double dx = odoPose.getX() - lastOdoPose.getX();
        double dy = odoPose.getY() - lastOdoPose.getY();

        /*if (dx < 0.01) dx = 0;

        if (dy < 0.01) dy = 0;


         */
        // Apply delta to FILTERED state
        estimatedPose = new Pose(
                estimatedPose.getX() + dx,
                estimatedPose.getY() + dy,
                heading
        );


        lastOdoPose = odoPose;

        // Increase positional uncertainty
        varX += processNoise * dt;
        varY += processNoise * dt;

        // ----------------------------
        // ANGULAR VELOCITY CHECK
        // ----------------------------

        double angularVelocity =
                (heading - lastHeading) / dt;

        lastHeading = heading;

        if (Math.abs(angularVelocity) > ANGULAR_VEL_THRESHOLD) {
            return; // reject vision
        }

        // ----------------------------
        // VISION UPDATE (MT2 X/Y ONLY)
        // ----------------------------

        limelight.updateRobotOrientation(Math.toDegrees(heading) + 90);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose convertedBotPose = from3DToPedro(result.getBotpose_MT2(), heading);

            double visionX =
                    convertedBotPose.getX();

            double visionY =
                    convertedBotPose.getY();

            // Kalman gain
            double kX = varX / (varX + visionNoise);
            double kY = varY / (varY + visionNoise);

            // Fuse into persistent state
            double fusedX = estimatedPose.getX()
                    + kX * (visionX - estimatedPose.getX());

            double fusedY = estimatedPose.getY()
                    + kY * (visionY - estimatedPose.getY());

            estimatedPose = new Pose(
                    fusedX,
                    fusedY,
                    heading
            );

            // Reduce covariance
            varX *= (1 - kX);
            varY *= (1 - kY);
        }
    }

    // ------------------------------------------------

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
    public void resetIMU() throws InterruptedException { }

    @Override
    public double getIMUHeading() {
        return pinpointLocalizer.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return pinpointLocalizer.isNAN();
    }

    public void recalibrate() throws InterruptedException{
        pinpointLocalizer.recalibrate();
    }


    public Pose getPose() {
        return estimatedPose;
    }
}