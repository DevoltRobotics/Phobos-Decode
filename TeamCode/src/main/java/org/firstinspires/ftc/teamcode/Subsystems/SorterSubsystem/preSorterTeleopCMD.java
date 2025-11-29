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

        timer= new ElapsedTime();

        addRequirements(sorterSubsystem);
    }

    @Override
    public void execute() {

        if (sensorsSb.sorterMode) {

            if (timer.milliseconds() > 600 && sorterSubsystem.blockersStatus.equals(SorterSubsystem.BlockersStatus.oneOpen)) {
                new lateralBlockersCMD(sorterSubsystem, 0, 0).schedule();

            }

            if (sensorsSb.rightDetected || sensorsSb.leftDetected) {
                if (sensorsSb.rightArtifact.equals(sensorsSb.targetArtifact)) {
                    timer.reset();
                    new lateralBlockersCMD(sorterSubsystem, blockersUp, 0).schedule();

                } else if (sensorsSb.leftArtifact.equals(sensorsSb.targetArtifact)) {
                    timer.reset();
                    new lateralBlockersCMD(sorterSubsystem, 0, blockersUp).schedule();

                }
            }
        }
    }

}
