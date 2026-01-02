package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;

public class preSorterTeleopCMD extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final SensorsSubsystem sensorsSb;

    private final ElapsedTime timer;

    public preSorterTeleopCMD(SorterSubsystem sorterSb, SensorsSubsystem sensorsSubsystem) {
        sorterSubsystem = sorterSb;
        sensorsSb = sensorsSubsystem;

        timer = new ElapsedTime();

        addRequirements(sorterSubsystem);
    }

    @Override
    public void execute() {

        if (!sorterSubsystem.isShooting) {

            if (sensorsSb.sorterMode) {

                if (timer.milliseconds() > 1200 && !sorterSubsystem.blockersStatus.equals(SorterSubsystem.BlockersStatus.closed)) {
                    sorterSubsystem.setLateralPositions(0, 0);

                }

                if (sensorsSb.rightDetected || sensorsSb.leftDetected) {
                    if (!(sensorsSb.currentRightArtifact == null) && sensorsSb.currentRightArtifact.equals(sensorsSb.targetArtifact)) {
                        timer.reset();
                        sorterSubsystem.setLateralPositions(blockersUp, 0);

                    } else if (!(sensorsSb.currentLeftArtifact == null) && sensorsSb.currentLeftArtifact.equals(sensorsSb.targetArtifact)) {
                        timer.reset();
                        sorterSubsystem.setLateralPositions(0, blockersUp);

                    }
                }
            }
        }


    }
}
