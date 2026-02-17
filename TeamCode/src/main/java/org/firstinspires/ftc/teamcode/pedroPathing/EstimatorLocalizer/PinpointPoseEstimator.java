package org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.NavigableMap;
import java.util.TreeMap;

public class PinpointPoseEstimator {

    private static final double BUFFER_DURATION = 1.5;

    private final ElapsedTime timer = new ElapsedTime();

    private Pose estimatedPose;

    private final double qx, qy;
    private final double kx, ky;

    private final NavigableMap<Double, Pose> odometryBuffer = new TreeMap<>();
    private final NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

    public PinpointPoseEstimator(
            Pose initialPose,
            double stateStdDevX,
            double stateStdDevY,
            double visionStdDevX,
            double visionStdDevY
    ) {
        this.estimatedPose = initialPose;

        qx = stateStdDevX * stateStdDevX;
        qy = stateStdDevY * stateStdDevY;

        double rx = visionStdDevX * visionStdDevX;
        double ry = visionStdDevY * visionStdDevY;

        kx = qx == 0 ? 0 : qx / (qx + Math.sqrt(qx * rx));
        ky = qy == 0 ? 0 : qy / (qy + Math.sqrt(qy * ry));

        timer.reset();
    }

    // ------------------------------------------------
    // CALL EVERY LOOP
    // ------------------------------------------------
    public Pose update(Pose odometryPose) {

        double now = timer.seconds();

        odometryBuffer.put(now, odometryPose);
        cleanOldOdometry(now);

        estimatedPose = odometryPose;

        if (!visionUpdates.isEmpty()) {
            VisionUpdate latest = visionUpdates.lastEntry().getValue();
            estimatedPose = latest.compensate(odometryPose);
        }

        return estimatedPose;
    }

    // ------------------------------------------------
    // ADD VISION MEASUREMENT (latency in seconds)
    // ------------------------------------------------
    public void addVisionMeasurement(
            Pose visionPose,
            double latencySeconds
    ) {

        double now = timer.seconds();
        double timestamp = now - latencySeconds;

        if (odometryBuffer.isEmpty()) return;

        if (odometryBuffer.lastKey() - BUFFER_DURATION > timestamp) {
            return;
        }

        cleanOldVision();

        Pose odometrySample = sampleOdometry(timestamp);
        if (odometrySample == null) return;

        Pose estimatedSample = sampleAt(timestamp);
        if (estimatedSample == null) return;

        double dx = visionPose.getX() - estimatedSample.getX();
        double dy = visionPose.getY() - estimatedSample.getY();

        double scaledDx = kx * dx;
        double scaledDy = ky * dy;

        Pose corrected = new Pose(
                estimatedSample.getX() + scaledDx,
                estimatedSample.getY() + scaledDy,
                estimatedSample.getHeading()
        );

        VisionUpdate update = new VisionUpdate(corrected, odometrySample);

        visionUpdates.put(timestamp, update);
        visionUpdates.tailMap(timestamp, false).clear();

        estimatedPose = update.compensate(
                odometryBuffer.lastEntry().getValue()
        );
    }

    // ------------------------------------------------

    private Pose sampleOdometry(double timestamp) {
        return odometryBuffer.floorEntry(timestamp) != null
                ? odometryBuffer.floorEntry(timestamp).getValue()
                : null;
    }

    private Pose sampleAt(double timestamp) {

        if (visionUpdates.isEmpty()
                || timestamp < visionUpdates.firstKey()) {
            return sampleOdometry(timestamp);
        }

        VisionUpdate update =
                visionUpdates.floorEntry(timestamp).getValue();

        Pose odo = sampleOdometry(timestamp);
        return odo == null ? null : update.compensate(odo);
    }

    private void cleanOldOdometry(double now) {
        odometryBuffer.headMap(now - BUFFER_DURATION).clear();
    }

    private void cleanOldVision() {
        if (odometryBuffer.isEmpty()) return;
        double oldestTime = odometryBuffer.firstKey();
        visionUpdates.headMap(oldestTime, false).clear();
    }

    public Pose getEstimatedPose() {
        return estimatedPose;
    }

    // ------------------------------------------------

    private static class VisionUpdate {

        private final Pose visionPose;
        private final Pose odometryPose;

        public VisionUpdate(Pose visionPose, Pose odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        public Pose compensate(Pose currentOdometry) {

            double dx = currentOdometry.getX() - odometryPose.getX();
            double dy = currentOdometry.getY() - odometryPose.getY();

            return new Pose(
                    visionPose.getX() + dx,
                    visionPose.getY() + dy,
                    currentOdometry.getHeading()
            );
        }
    }
}