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

    private final ElapsedTime deadTimer = new ElapsedTime();
    private boolean detected = false;

    public preSorterCmd(SorterSubsystem sorterSb, SensorsSubsystem sensorsSubsystem, VisionSubsystem visionSubsystem) {
        sorterSubsystem = sorterSb;
        sensorsSb = sensorsSubsystem;

        visionSb = visionSubsystem;


        addRequirements(sensorsSb, sorterSubsystem);
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
                        sorterSubsystem.setPositions(blockersUp, 0);
                        detected = true;
                        cancel();
                    } else if (Artifact.Green.equals(sensorsSb.leftArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);
                        detected = true;
                    }
                    break;

                case PPG:
                case PGP:
                    if (Artifact.Purple.equals(sensorsSb.rightArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);
                        detected = true;
                    } else if (Artifact.Purple.equals(sensorsSb.leftArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);
                        detected = true;
                    }
                    break;

            }

        }

    }

    @Override
    public void end(boolean interrupted) {
        if (!detected) {
            sorterSubsystem.setPositions(blockersUp, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return deadTimer.seconds() > 0.5 || detected;
    }
}
