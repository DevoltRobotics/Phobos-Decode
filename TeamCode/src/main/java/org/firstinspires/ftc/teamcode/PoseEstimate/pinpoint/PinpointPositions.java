package org.firstinspires.ftc.teamcode.PoseEstimate.pinpoint;

import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.Interpolatable;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.MathUtil;

public class PinpointPositions implements Interpolatable<PinpointPositions> {

    public double x, y, rot;

    public PinpointPositions(double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    public PinpointPositions() {
        this(0, 0, 0);
    }

    @Override
    public PinpointPositions interpolate(PinpointPositions endValue, double t) {
        return new PinpointPositions(
                MathUtil.interpolate(x, endValue.x, t),
                MathUtil.interpolate(y, endValue.y, t),
                MathUtil.interpolate(rot, endValue.rot, t)
        );
    }
}
