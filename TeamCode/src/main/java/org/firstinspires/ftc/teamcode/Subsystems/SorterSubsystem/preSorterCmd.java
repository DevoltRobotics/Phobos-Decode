package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.BlockersStatus.closed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;

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

            sensorsSb.realRightArtifact = sensorsSb.currentRightArtifact;
            sensorsSb.realleftArtifact = sensorsSb.currentLeftArtifact;

            switch (visionSb.pattern) {
                case GPP:
                    if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);
                        detected = true;

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RR;

                    } else if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setPositions(0, blockersUp);
                        detected = true;

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LL;

                    }

                    break;

                case PGP:

                    if (Artifact.Purple.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);
                        detected = true;


                    } else if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setPositions(0, blockersUp);
                        detected = true;
                    }


                    if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LR;
                    } else if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RL;

                    }

                    break;

                case PPG:
                    if (Artifact.Purple.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);

                        if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RR;

                        }else {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RL;

                        }

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {

                        sorterSubsystem.setPositions(0, blockersUp);

                        if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LL;

                        }else {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LR;

                        }

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
        return deadTimer.seconds() > 0.4 || detected;
    }
}
