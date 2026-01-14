package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class turretToPosCMD extends CommandBase {

    TurretSubsystem turretSb;

    protected Double targetPos;


    public turretToPosCMD(TurretSubsystem turretSb, Double targetPos) {
        this.turretSb = turretSb;

        this.targetPos = targetPos;

        addRequirements(this.turretSb);
    }

    @Override
    public void execute() {
        turretSb.turretTarget = targetPos;

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
