package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;

public class lateralBlockersCMD extends CommandBase {

    private final SorterSubsystem intakeSubsystem;

    private double rightPos;
    private double leftPos;

    public lateralBlockersCMD(SorterSubsystem sorterSb, double rightPosTarget) {
        intakeSubsystem = sorterSb;
        rightPos = rightPosTarget;

        addRequirements(intakeSubsystem);
    }

    public lateralBlockersCMD(SorterSubsystem sorterSb, double rightPosTarget, double leftPosTarget) {
        intakeSubsystem = sorterSb;
        rightPos = rightPosTarget;
        leftPos = leftPosTarget;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {

        intakeSubsystem.blockerR.setPosition(rightPos);
        intakeSubsystem.blockerL.setPosition(leftPos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
