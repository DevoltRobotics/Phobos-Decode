package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Utilities.Pattern;

public class setIsCorner extends CommandBase {

    private final VisionSubsystem visionSubsystem;

    public setIsCorner(VisionSubsystem vSb) {
        visionSubsystem = vSb;

        addRequirements(visionSubsystem);
    }


    @Override
    public void execute() {

        visionSubsystem.currentArtifactCorner = visionSubsystem.isArtifactsCorner();
    }


}
