package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.BlockersStatus.closed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;

import java.util.function.BooleanSupplier;

public class preSorterTeleopCMD extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final SensorsSubsystem sensorsSb;

    private final ElapsedTime deadTimer = new ElapsedTime();
    private boolean detected = false;

    private double deadTime;

    public preSorterTeleopCMD(SorterSubsystem sorterSb, SensorsSubsystem sensorsSubsystem, double deadTime) {
        sorterSubsystem = sorterSb;
        sensorsSb = sensorsSubsystem;

        this.deadTime = deadTime;

        addRequirements(sensorsSb, sorterSubsystem);
    }

    @Override
    public void execute() {

        if ((sensorsSb.rightDetected || sensorsSb.leftDetected) && sorterSubsystem.blockersStatus.equals(closed)) {

            switch (visionSb.pattern) {
                case GPP:
                    if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setLateralPositions(blockersUp, 0);

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RR;

                        detected = true;
                    } else if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setLateralPositions(0, blockersUp);

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LL;

                        detected = true;
                    } else if (Artifact.Purple.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setLateralPositions(0, blockersUp);

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LL;

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setLateralPositions(blockersUp, 0);

                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RR;
                        detected = true;
                    }

                    break;

                case PGP:
                    if (Artifact.Purple.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setLateralPositions(blockersUp, 0);

                        if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RL;

                        }

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setLateralPositions(0, blockersUp);

                        if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LR;

                        }

                        detected = true;

                    }else if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {

                        sorterSubsystem.setLateralPositions(0, blockersUp);
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LR;

                        detected = true;

                    }else if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {

                        sorterSubsystem.setLateralPositions(blockersUp, 0);
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RL;

                        detected = true;
                    }

                    break;

                case PPG:

                    if (Artifact.Green.equals(sensorsSb.currentRightArtifact)) {

                        sorterSubsystem.setLateralPositions(0, blockersUp);
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.LL;

                        detected = true;

                    }else if (Artifact.Green.equals(sensorsSb.currentLeftArtifact)) {

                        sorterSubsystem.setLateralPositions(blockersUp, 0);
                        sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RR;

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentRightArtifact)) {
                        sorterSubsystem.setLateralPositions(blockersUp, 0);

                        if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                            sensorsSb.relaseOrder = SensorsSubsystem.RelaseOrder.RL;

                        }

                        detected = true;

                    } else if (Artifact.Purple.equals(sensorsSb.currentLeftArtifact)) {
                        sorterSubsystem.setLateralPositions(0, blockersUp);

                        detected = true;

                    }

                    break;

            }

        }


        /*
        if (!sorterSubsystem.isShooting) {
            if (preparingShoot.getAsBoolean()) {
                if (sensorsSb.sorterMode) {

                    intaking = gamepad.right_trigger > 0.3;

                    if (sensorsSb.rightDetected || sensorsSb.leftDetected) {

                        if (!(sensorsSb.currentRightArtifact == null) && sensorsSb.currentRightArtifact.equals(sensorsSb.targetArtifact)) {
                            sorterSubsystem.setLateralPositions(0, blockersUp);

                        } else if (!(sensorsSb.currentLeftArtifact == null) && sensorsSb.currentLeftArtifact.equals(sensorsSb.targetArtifact)) {
                            sorterSubsystem.setLateralPositions(blockersUp, 0);

                        } else if (!intaking) {
                            sorterSubsystem.setLateralPositions(blockersUp, blockersUp);

                        }

                    }
                }
            }
        }

         */


    }
}
