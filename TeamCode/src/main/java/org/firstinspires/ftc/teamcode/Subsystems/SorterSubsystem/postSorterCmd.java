package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

public class postSorterCmd extends SequentialCommandGroup {

    public static int firstArtifactWaitTimer = 1000;
    public static int waitUntilArtifactsTimer = 1400;

    public postSorterCmd(SorterSubsystem sorterSubsystem, Pattern pattern, Artifact leftArtifact, Artifact rightArtifact) {

        switch (pattern) {
            case PPG:
                addCommands(

                        new ConditionalCommand(
                                new lateralBlockersCMD(sorterSubsystem, 0, blockersUp),
                                new lateralBlockersCMD(sorterSubsystem, blockersUp, 0),
                                () -> leftArtifact.equals(Artifact.Purple)
                        ),

                        new WaitCommand(waitUntilArtifactsTimer)
                );
                break;
            case PGP:
                addCommands(
                        new ConditionalCommand(
                                new lateralBlockersCMD(sorterSubsystem, 0, blockersUp),
                                new lateralBlockersCMD(sorterSubsystem, blockersUp, 0),
                                () -> leftArtifact.equals(Artifact.Green)
                        ),

                        new WaitCommand(waitUntilArtifactsTimer)
                );
                break;
            case GPP:

                break;
        }
    }

}
