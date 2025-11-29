package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

public class lightTimeCMD extends CommandBase {

    private final SensorsSubsystem sensorsSubsystem;

    private final ElapsedTime timer;

    private double position;
    private int deadTime;

    public lightTimeCMD(SensorsSubsystem sensorsSb, double position, int deadTime) {
        sensorsSubsystem = sensorsSb;

        this.position = position;
        this.deadTime = deadTime;

        timer = new ElapsedTime();
        addRequirements(sensorsSubsystem);
    }


    @Override
    public void execute() {

        sensorsSubsystem.light.setPosition(position);

    }

    @Override
    public void end(boolean interrupted) {
        sensorsSubsystem.light.setPosition(0);

    }

    @Override
    public boolean isFinished() {
        return timer.seconds() > deadTime;

    }
}
