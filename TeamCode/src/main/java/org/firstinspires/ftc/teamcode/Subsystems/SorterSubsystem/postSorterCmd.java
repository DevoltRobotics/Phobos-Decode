package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

public class postSorterCmd extends SequentialCommandGroup {

    public static int firstArtifactWaitTimer = 1000;
    public static int waitUntilArtifactsTimer = 1400;

    SorterSubsystem sorterSb;
    VisionSubsystem visionSb;

    public postSorterCmd(SorterSubsystem sorterSb, SensorsSubsystem sensorsSb, VisionSubsystem visionSb) {

        this.sorterSb = sorterSb;
        this.visionSb = visionSb;

        switch (visionSb.pattern) {
            case PPG:
                addCommands(

                        new ConditionalCommand(
                                new lateralBlockersCMD(sorterSb, 0, blockersUp),
                                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                                () -> sensorsSb.leftArtifact.equals(Artifact.Purple)
                        ),

                        new WaitCommand(waitUntilArtifactsTimer)
                );
                break;
            case PGP:
                addCommands(
                        new ConditionalCommand(
                                new lateralBlockersCMD(sorterSb, 0, blockersUp),
                                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                                () -> sensorsSb.leftArtifact.equals(Artifact.Green)
                        ),

                        new WaitCommand(waitUntilArtifactsTimer)
                );
                break;
            case GPP:

                break;
        }

        addRequirements(this.sorterSb);
    }

}
