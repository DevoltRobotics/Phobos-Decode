package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;

public class preSorterTeleopCMD extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final SensorsSubsystem sensorsSb;

    private Boolean intaking = false;


    private Gamepad gamepad;

    public preSorterTeleopCMD(SorterSubsystem sorterSb, SensorsSubsystem sensorsSubsystem, Gamepad gamepad) {
        sorterSubsystem = sorterSb;
        sensorsSb = sensorsSubsystem;

        this.gamepad = gamepad;

        addRequirements(sorterSubsystem);
    }

    @Override
    public void execute() {

        if (!sorterSubsystem.isShooting) {

            if (sensorsSb.sorterMode) {

                intaking = gamepad.right_trigger > 0.3;

                if (sensorsSb.rightDetected || sensorsSb.leftDetected) {

                    if (!(sensorsSb.currentRightArtifact == null) && sensorsSb.currentRightArtifact.equals(sensorsSb.targetArtifact)) {
                        sorterSubsystem.setLateralPositions(blockersUp, 0);

                    } else if (!(sensorsSb.currentLeftArtifact == null) && sensorsSb.currentLeftArtifact.equals(sensorsSb.targetArtifact)) {
                        sorterSubsystem.setLateralPositions(0, blockersUp);

                    } else if (!intaking) {
                        sorterSubsystem.setLateralPositions(0, 0);

                    }

                }
            }
        }


    }
}
