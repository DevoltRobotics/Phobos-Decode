package org.firstinspires.ftc.teamcode.PoseEstimate.pinpoint;

import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.Odometry;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Rotation2d;

public class PinpointOdometry extends Odometry<PinpointPositions> {
    /**
     * Constructs an Odometry object.
     *
     * @param kinematics        The kinematics of the drivebase.
     * @param gyroAngle         The angle reported by the gyroscope.
     * @param wheelPositions    The current encoder readings.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public PinpointOdometry(PinpointKinematics kinematics, Rotation2d gyroAngle, PinpointPositions wheelPositions, Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
    }
}
