package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class turretToPosCMD extends CommandBase {

    TurretSubsystem turretSb;

    protected Double targetPos;
    double error;

    boolean finishOnSetpoint = false;

    public turretToPosCMD(TurretSubsystem turretSb, Double targetPos, boolean finishOnSetpoint) {
        this.turretSb = turretSb;

        this.targetPos = targetPos;

        this.finishOnSetpoint = finishOnSetpoint;

        addRequirements(this.turretSb);
    }

    public turretToPosCMD(TurretSubsystem subsystem, Double targetPos) {
        this(subsystem, targetPos, true);
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
