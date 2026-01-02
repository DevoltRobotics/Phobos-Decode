package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import com.seattlesolvers.solverslib.command.CommandBase;

public class lateralBlockersCMD extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final double rightPos;
    private final double leftPos;

    public lateralBlockersCMD(SorterSubsystem sorterSb, double rightPosTarget, double leftPosTarget) {
        sorterSubsystem = sorterSb;
        rightPos = rightPosTarget;
        leftPos = leftPosTarget;

        addRequirements(sorterSubsystem);
    }

    @Override
    public void execute() {
        sorterSubsystem.setLateralPositions(rightPos, leftPos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
