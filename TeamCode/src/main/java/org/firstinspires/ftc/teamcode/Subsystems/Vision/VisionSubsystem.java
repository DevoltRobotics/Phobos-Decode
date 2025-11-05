package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    public Limelight3A limelight;

    public enum Pattern{
        GPP, PGP, PPG

    }

    public static Pattern pattern;

    public VisionSubsystem(HardwareMap hMap) {
        limelight = hMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();
    }

    @Override
    public void periodic() {
    }
}
