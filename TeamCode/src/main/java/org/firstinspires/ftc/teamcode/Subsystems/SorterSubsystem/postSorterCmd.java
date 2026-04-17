package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

public class postSorterCmd extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final SensorsSubsystem sensorsSb;

    private final ElapsedTime deadTimer = new ElapsedTime();
    private boolean detected = false;

    public postSorterCmd(SorterSubsystem sorterSb, SensorsSubsystem sensorsSubsystem) {
        sorterSubsystem = sorterSb;
        sensorsSb = sensorsSubsystem;

        addRequirements(sensorsSb, sorterSubsystem);
    }

    @Override
    public void initialize() {
        deadTimer.reset();
    }

    @Override
    public void execute() {

        if (sensorsSb.relaseOrder != null) {
            switch (sensorsSb.relaseOrder) {
                case RL:
                case LL:
                    sorterSubsystem.setLateralPositions(blockersUp, 0);

                    break;

                case LR:
                case RR:
                    sorterSubsystem.setLateralPositions(0, blockersUp);

                    break;

            }
        }


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
