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

    private double deadTime;

    public preSorterCmd(SorterSubsystem sorterSb, SensorsSubsystem sensorsSubsystem, VisionSubsystem visionSubsystem, double deadTime) {
        sorterSubsystem = sorterSb;
        sensorsSb = sensorsSubsystem;

        visionSb = visionSubsystem;

        this.deadTime = deadTime;

        addRequirements(sensorsSb, sorterSubsystem);
    }

    @Override
    public void initialize() {
        deadTimer.reset();

        sensorsSb.relaseOrder = null;
    }

    @Override
    public void execute() {


        if ((sensorsSb.rightDetected || sensorsSb.leftDetected) && sorterSubsystem.blockersStatus.equals(closed)) {

            switch (visionSb.pattern) {
                case GPP:
                    if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RR;

                        detected = true;
                    } else if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setPositions(0, blockersUp);

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LL;

                        detected = true;
                    } else if (Artifact.Purple.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setPositions(0, blockersUp);

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LL;

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RR;
                        detected = true;
                    }

                    break;

                case PGP:
                    if (Artifact.Purple.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);

                        if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RL;

                        }

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setPositions(0, blockersUp);

                        if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LR;

                        }

                        detected = true;

                    }else if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {

                        sorterSubsystem.setPositions(0, blockersUp);
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LR;

                        detected = true;

                    }else if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {

                        sorterSubsystem.setPositions(blockersUp, 0);
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RL;

                        detected = true;
                    }

                    break;

                case PPG:

                    if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {

                        sorterSubsystem.setPositions(0, blockersUp);
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LL;

                        detected = true;

                    }else if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {

                        sorterSubsystem.setPositions(blockersUp, 0);
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RR;

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setPositions(blockersUp, 0);

                        if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RL;

                        }

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setPositions(0, blockersUp);

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
        return deadTimer.seconds() > deadTime || detected;
    }
}
