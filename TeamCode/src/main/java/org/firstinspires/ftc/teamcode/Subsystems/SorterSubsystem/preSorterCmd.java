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

public class preSorterCmd extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final SensorsSubsystem sensorsSb;

    private final VisionSubsystem visionSb;


    private final ElapsedTime deadTimer;

    public preSorterCmd(SorterSubsystem sorterSb, SensorsSubsystem sensorsSubsystem, VisionSubsystem visionSubsystem) {
        sorterSubsystem = sorterSb;
        sensorsSb = sensorsSubsystem;

        visionSb = visionSubsystem;


        deadTimer = new ElapsedTime();

        addRequirements(sensorsSb);
    }

    @Override
    public void initialize() {
        deadTimer.reset();
    }

    @Override
    public void execute() {

        if ((sensorsSb.rightDetected || sensorsSb.leftDetected) && sorterSubsystem.blockersStatus.equals(closed)) {

            switch (visionSb.pattern) {

                case GPP:
                    if (Artifact.Green.equals(sensorsSb.rightArtifact)) {
                        new lateralBlockersCMD(sorterSubsystem, blockersUp, 0).schedule();

                    } else if (Artifact.Green.equals(sensorsSb.leftArtifact)) {
                        new lateralBlockersCMD(sorterSubsystem, 0, blockersUp).schedule();

                    }

                    break;


                case PGP:
                    if (Artifact.Purple.equals(sensorsSb.rightArtifact)) {
                        new lateralBlockersCMD(sorterSubsystem, blockersUp, 0).schedule();

                    } else if (Artifact.Purple.equals(sensorsSb.leftArtifact)) {
                        new lateralBlockersCMD(sorterSubsystem, 0, blockersUp).schedule();

                    }

                    break;

                case PPG:
                    if (Artifact.Purple.equals(sensorsSb.rightArtifact)) {
                        new lateralBlockersCMD(sorterSubsystem, blockersUp, 0).schedule();

                    } else if (Artifact.Purple.equals(sensorsSb.leftArtifact)) {
                        new lateralBlockersCMD(sorterSubsystem, 0, blockersUp).schedule();

                    }

                    break;


            }

        }

    }

    @Override
    public void end(boolean interrupted) {
        if (sorterSubsystem.blockersStatus.equals(closed)){
            new lateralBlockersCMD(sorterSubsystem, blockersUp, 0).schedule();

        }

    }

    @Override
    public boolean isFinished() {
        return deadTimer.milliseconds() > 200 || sorterSubsystem.blockersStatus.equals(oneOpen);
    }
}
