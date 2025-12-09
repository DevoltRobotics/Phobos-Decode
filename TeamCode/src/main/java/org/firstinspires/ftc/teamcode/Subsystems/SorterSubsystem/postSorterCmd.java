package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.BlockersStatus.closed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.BlockersStatus.oneOpen;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

public class postSorterCmd extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final SensorsSubsystem sensorsSb;

    private final VisionSubsystem visionSb;

    private final ElapsedTime deadTimer;

    boolean hasMoved = false;
    public postSorterCmd(SorterSubsystem sorterSb, SensorsSubsystem sensorsSubsystem, VisionSubsystem visionSubsystem) {
        sorterSubsystem = sorterSb;
        sensorsSb = sensorsSubsystem;

        visionSb = visionSubsystem;


        deadTimer = new ElapsedTime();

        addRequirements(sorterSubsystem);
    }

    @Override
    public void execute() {

        if ((sensorsSb.rightDetected || sensorsSb.leftDetected)) {

            switch (visionSb.pattern) {

                case GPP:
                    if (sensorsSb.rightArtifact.equals(Artifact.Purple)) {
                        new lateralBlockersCMD(sorterSubsystem, blockersUp, 0).schedule();

                        hasMoved = true;
                    } else if (sensorsSb.leftArtifact.equals(Artifact.Purple)) {
                        new lateralBlockersCMD(sorterSubsystem, 0, blockersUp).schedule();

                        hasMoved = true;
                    }

                    break;


                case PGP:
                    if (sensorsSb.rightArtifact.equals(Artifact.Green)) {
                        new lateralBlockersCMD(sorterSubsystem, blockersUp, 0).schedule();

                        hasMoved = true;
                    } else if (sensorsSb.leftArtifact.equals(Artifact.Green)) {
                        new lateralBlockersCMD(sorterSubsystem, 0, blockersUp).schedule();

                        hasMoved = true;
                    }

                    break;

                case PPG:

                    if (sensorsSb.rightArtifact.equals(Artifact.Purple)) {
                        new lateralBlockersCMD(sorterSubsystem, blockersUp, 0).schedule();

                        hasMoved = true;
                    } else if (sensorsSb.leftArtifact.equals(Artifact.Purple)) {
                        new lateralBlockersCMD(sorterSubsystem, 0, blockersUp).schedule();

                        hasMoved = true;
                    }

                    break;


            }

        }

    }

    @Override
    public boolean isFinished() {
        return deadTimer.seconds() > 300 || hasMoved;
    }
}
