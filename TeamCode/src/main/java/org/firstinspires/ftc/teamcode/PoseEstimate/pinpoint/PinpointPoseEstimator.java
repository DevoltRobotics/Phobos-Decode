package org.firstinspires.ftc.teamcode.PoseEstimate.pinpoint;

import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.Matrix;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.Odometry;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.PoseEstimator;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.numbers.N1;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.numbers.N3;

public class PinpointPoseEstimator extends PoseEstimator<PinpointPositions> {
    /**
     * Constructs a PoseEstimator.
     *
     * @param odometry                 A correctly-configured odometry object for your drivetrain.
     * @param stateStdDevs             Standard deviations of the pose estimate (x position in meters, y position
     *                                 in meters, and heading in radians). Increase these numbers to trust your state estimate
     *                                 less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *                                 in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public PinpointPoseEstimator(Odometry<PinpointPositions> odometry, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(odometry, stateStdDevs, visionMeasurementStdDevs);
    }
}
