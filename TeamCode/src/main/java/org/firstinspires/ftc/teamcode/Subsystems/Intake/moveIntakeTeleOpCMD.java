package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;

public class moveIntakeTeleOpCMD extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    private double inPower;
    private double transPower;

    public moveIntakeTeleOpCMD(IntakeSubsystem intakeSb, double powerTarget) {
        intakeSubsystem = intakeSb;
        inPower = powerTarget;

        transPower = powerTarget;

        addRequirements(intakeSubsystem);
    }

    public moveIntakeTeleOpCMD(IntakeSubsystem intakeSb, double powerTarget, double transferTarget) {
        intakeSubsystem = intakeSb;
        inPower = powerTarget;

        transPower = transferTarget;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakePower = inPower;

        intakeSubsystem.transferPower = transPower;
    }


    @Override
    public void end(boolean interruptible) {
        intakeSubsystem.intakePower = 0;
        intakeSubsystem.transferPower = 0;
    }
}
