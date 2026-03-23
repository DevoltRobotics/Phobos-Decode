package org.firstinspires.ftc.teamcode.PoseEstimate.pinpoint;

import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.Kinematics;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Twist2d;

public class PinpointKinematics implements Kinematics<PinpointSpeeds, PinpointPositions> {
    @Override
    public ChassisSpeeds toChassisSpeeds(PinpointSpeeds wheelSpeeds) {
        return new ChassisSpeeds(wheelSpeeds.xSpeed, wheelSpeeds.ySpeed, wheelSpeeds.rotSpeed);
    }

    @Override
    public PinpointSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        return new PinpointSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public Twist2d toTwist2d(PinpointPositions start, PinpointPositions end) {
        return new Twist2d(end.x - start.x, end.y - start.y, Rotation2d.fromRadians(end.rot).rotateBy(Rotation2d.fromRadians(start.rot)).getRadians());
    }

    @Override
    public PinpointPositions copy(PinpointPositions positions) {
        return new PinpointPositions(positions.x, positions.y, positions.rot);
    }

    @Override
    public void copyInto(PinpointPositions positions, PinpointPositions output) {
        output.x = positions.x;
        output.y = positions.y;
        output.rot = positions.rot;
    }

    @Override
    public PinpointPositions interpolate(PinpointPositions startValue, PinpointPositions endValue, double t) {
        return startValue.interpolate(endValue, t);
    }
}
