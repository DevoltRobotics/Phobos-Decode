package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import com.seattlesolvers.solverslib.command.CommandBase;

public class horizontalBlockerCMD extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final double targetPos;

    public horizontalBlockerCMD(SorterSubsystem sorterSb, double target) {
        sorterSubsystem = sorterSb;
        targetPos = target;

        addRequirements(sorterSubsystem);
    }

    @Override
    public void execute() {

        sorterSubsystem.setHorizontalPos(targetPos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
