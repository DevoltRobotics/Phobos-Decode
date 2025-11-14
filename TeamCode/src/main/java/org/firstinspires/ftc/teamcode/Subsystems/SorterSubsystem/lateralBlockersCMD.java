package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import com.seattlesolvers.solverslib.command.CommandBase;

public class lateralBlockersCMD extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private double rightPos;
    private double leftPos;

    public lateralBlockersCMD(SorterSubsystem sorterSb, double rightPosTarget) {
        sorterSubsystem = sorterSb;
        rightPos = rightPosTarget;

        addRequirements(sorterSubsystem);
    }

    public lateralBlockersCMD(SorterSubsystem sorterSb, double rightPosTarget, double leftPosTarget) {
        sorterSubsystem = sorterSb;
        rightPos = rightPosTarget;
        leftPos = leftPosTarget;

        addRequirements(sorterSubsystem);
    }

    @Override
    public void execute() {

        sorterSubsystem.blockerR.setPosition(0.5 + rightPos);
        sorterSubsystem.blockerL.setPosition(0.5 - leftPos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
