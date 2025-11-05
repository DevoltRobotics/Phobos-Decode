package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import static org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandBase;

public class PatternSelectCMD extends CommandBase {

    private final VisionSubsystem visionSubsystem;
    private final Limelight3A limelight;

    private int detectedTag = 0;

    public PatternSelectCMD(VisionSubsystem vSb) {
        visionSubsystem = vSb;

        limelight = visionSubsystem.limelight;

        addRequirements(vSb);
    }


    @Override
    public void initialize() {
        visionSubsystem.limelight.pipelineSwitch(0);

        super.initialize();
    }


    @Override
    public void execute() {

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                detectedTag = result.getPipelineIndex();

            }

        }

        switch (detectedTag){
            case 1:
                pattern = Pattern.GPP;

            case 2:
                pattern = Pattern.PGP;

            case 3:
                pattern = Pattern.PPG;

        }
    }

    @Override
    public boolean isFinished() {
        return limelight.getLatestResult().isValid();
    }
}
