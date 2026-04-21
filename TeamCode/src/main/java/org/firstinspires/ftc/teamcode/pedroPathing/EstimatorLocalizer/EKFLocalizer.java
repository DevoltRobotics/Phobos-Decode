package org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer;

import static org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer.ConversionUtil.from3DToPedro;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EKFLocalizer implements Localizer {

    private final PinpointLocalizer odo;
    private final Limelight3A ll;

    // State
    private double x, y, heading;

    // Covariance (diagonal EKF)
    private double varX = 0.2;
    private double varY = 0.2;
    private double varH = 0.1;

    private Pose lastOdoPose;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    // Noise tuning
    private final double processNoise = 0.02;
    private final double headingNoise = 0.01;

    private final double visionNoiseMT1 = 1.0; // absolute
    private final double visionNoiseMT2 = 0.3; // smooth

    private static final double SPEED_THRESHOLD = 10; // in/s
    private static final double MAX_VISION_JUMP = 4.0; // inches

    public EKFLocalizer(PinpointLocalizer odo, Limelight3A ll) {
        this.odo = odo;
        this.ll = ll;

        odo.getPinpoint().recalibrateIMU();

        Pose start = odo.getPose();
        x = start.getX();
        y = start.getY();
        heading = start.getHeading();

        lastOdoPose = start;
        timer.reset();
    }

    @Override
    public void update() {

        double now = timer.seconds();
        double dt = now - lastTime;
        lastTime = now;
        if (dt <= 0) dt = 1e-6;

        // ----------------------------
        // 🔁 PREDICTION (ODOMETRY)
        // ----------------------------

        odo.update();
        Pose odoPose = odo.getPose();

        double dx = odoPose.getX() - lastOdoPose.getX();
        double dy = odoPose.getY() - lastOdoPose.getY();
        double newHeading = odoPose.getHeading();

        x += dx;
        y += dy;
        heading = newHeading;

        lastOdoPose = odoPose;

        // Increase uncertainty
        varX += processNoise * dt;
        varY += processNoise * dt;
        varH += headingNoise * dt;

        // ----------------------------
        // 🚗 SPEED CHECK
        // ----------------------------

        Pose vel = odo.getVelocity();
        double speed = Math.hypot(vel.getX(), vel.getY());

        boolean useMT1 = speed < SPEED_THRESHOLD;

        // ----------------------------
        // 👁️ VISION UPDATE
        // ----------------------------

        ll.updateRobotOrientation(Math.toDegrees(heading) + 90);
        LLResult result = ll.getLatestResult();

        if (result == null || !result.isValid()) return;

        Pose visionPose;

        if (useMT1) {
            visionPose = from3DToPedro(result.getBotpose(), heading);
        } else {
            visionPose = from3DToPedro(result.getBotpose_MT2(), heading);
        }

        double visionX = visionPose.getX();
        double visionY = visionPose.getY();

        // ----------------------------
        // ❌ OUTLIER REJECTION
        // ----------------------------

        double error = Math.hypot(visionX - x, visionY - y);

        //if (error > MAX_VISION_JUMP) return;

        // ----------------------------
        // ⚖️ ADAPTIVE NOISE
        // ----------------------------

        double visionNoise = useMT1 ? visionNoiseMT1 : visionNoiseMT2;

        // ----------------------------
        // 📊 KALMAN GAIN
        // ----------------------------

        double kX = varX / (varX + visionNoise);
        double kY = varY / (varY + visionNoise);

        // ----------------------------
        // 🔄 UPDATE STATE
        // ----------------------------

        x += kX * (visionX - x);
        y += kY * (visionY - y);

        // Optional: heading fusion (only for MT1 + low speed)
        if (useMT1) {
            double visionHeading = visionPose.getHeading();
            double kH = varH / (varH + 0.5);

            heading += kH * angleWrap(visionHeading - heading);
            varH *= (1 - kH);
        }

        // ----------------------------
        // 📉 UPDATE COVARIANCE
        // ----------------------------

        varX *= (1 - kX);
        varY *= (1 - kY);
    }

    // ----------------------------
    // HELPERS
    // ----------------------------

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ----------------------------
    // REQUIRED METHODS
    // ----------------------------

    @Override
    public Pose getPose() {
        return new Pose(x, y, heading);
    }

    @Override
    public Pose getVelocity() {
        return odo.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return odo.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose pose) {
        odo.setStartPose(pose);
        x = pose.getX();
        y = pose.getY();
        heading = pose.getHeading();
    }

    @Override
    public void setPose(Pose pose) {
        odo.setPose(pose);
        x = pose.getX();
        y = pose.getY();
        heading = pose.getHeading();
    }

    @Override
    public double getTotalHeading() {
        return odo.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return odo.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return odo.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return odo.getTurningMultiplier();
    }

    @Override
    public void resetIMU() {}

    @Override
    public double getIMUHeading() {
        return odo.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return odo.isNAN();
    }
}
