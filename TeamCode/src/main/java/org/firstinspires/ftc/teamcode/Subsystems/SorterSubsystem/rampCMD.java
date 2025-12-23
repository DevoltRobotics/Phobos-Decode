package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import com.seattlesolvers.solverslib.command.CommandBase;

public class rampCMD extends CommandBase {

    private final SorterSubsystem sorterSubsystem;

    private final double pos;

    public rampCMD(SorterSubsystem sorterSb, double Pos) {
        sorterSubsystem = sorterSb;
        pos = Pos;

        addRequirements(sorterSubsystem);
    }

    @Override
    public void execute() {
        sorterSubsystem.setRampPos(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
