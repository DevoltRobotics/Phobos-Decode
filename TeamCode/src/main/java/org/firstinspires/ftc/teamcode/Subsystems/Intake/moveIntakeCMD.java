package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;

public class moveIntakeCMD extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    private double power;

    public moveIntakeCMD(IntakeSubsystem intakeSb, double powerTarget) {
        intakeSubsystem = intakeSb;
        power = powerTarget;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {

        intakeSubsystem.power = power;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
