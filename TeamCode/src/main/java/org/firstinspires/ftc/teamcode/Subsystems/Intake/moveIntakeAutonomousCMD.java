package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;

public class moveIntakeAutonomousCMD extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    private double inPower;

    private double transPower;

    public moveIntakeAutonomousCMD(IntakeSubsystem intakeSb, double powerTarget) {
        intakeSubsystem = intakeSb;
        inPower = powerTarget;

        transPower = powerTarget;


        addRequirements(intakeSubsystem);
    }

    public moveIntakeAutonomousCMD(IntakeSubsystem intakeSb, double inTarget, double transTarget) {
        intakeSubsystem = intakeSb;
        inPower = inTarget;

        transPower = transTarget;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakePower = inPower;
        intakeSubsystem.transferPower = transPower;
    }



    @Override
    public boolean isFinished() {
        return true;
    }
}
