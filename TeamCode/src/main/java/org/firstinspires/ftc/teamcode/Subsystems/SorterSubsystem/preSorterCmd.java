package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

public class preSorterCmd extends SequentialCommandGroup {

    public static int firstArtifactWaitTimer = 1000;
    public static int waitUntilArtifactsTimer = 1400;

    public preSorterCmd(SorterSubsystem sorterSubsystem, SensorsSubsystem sensorsSb, VisionSubsystem visionSb) {

        switch (visionSb.pattern) {
            case PPG:
                addCommands(
                        new ConditionalCommand(
                                new lateralBlockersCMD(sorterSubsystem, blockersUp, 0),
                                new lateralBlockersCMD(sorterSubsystem, 0, blockersUp),
                                () -> sensorsSb.rightArtifact.equals(Artifact.Purple)
                        ),
                        new WaitCommand(firstArtifactWaitTimer)

                );
                break;
            case PGP:
                addCommands(
                        new ConditionalCommand(
                                new lateralBlockersCMD(sorterSubsystem, blockersUp, 0),
                                new lateralBlockersCMD(sorterSubsystem, 0, blockersUp),
                                () -> sensorsSb.rightArtifact.equals(Artifact.Purple)
                        ),
                        new WaitCommand(firstArtifactWaitTimer)

                );
                break;
            case GPP:
                addCommands(
                        new ConditionalCommand(
                                new lateralBlockersCMD(sorterSubsystem, blockersUp, 0),
                                new lateralBlockersCMD(sorterSubsystem, 0, blockersUp),
                                () -> sensorsSb.rightArtifact.equals(Artifact.Green)
                        ),
                        new WaitCommand(waitUntilArtifactsTimer)
                );
                break;
        }
    }

}
