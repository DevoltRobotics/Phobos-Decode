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

        if(targetPos == null) {
            cancel();
            return;
        }

        double shortError = AngleUnit.normalizeDegrees(targetPos - turretSb.turretPRelative);
        double longError = shortError > 0
                ? shortError - 360
                : shortError + 360;

        double predictedShort = turretSb.turretPRelative + shortError;

        if(predictedShort >= TurretSubsystem.lowerLimit && predictedShort <= TurretSubsystem.upperLimit) {
            error = shortError;
        } else {
            error = longError;
        }

        turretSb.setTurretPower(turretSb.turretPid.calculate(error, 0));

        FtcDashboard.getInstance().getTelemetry().addData("turret target", targetPos);
    }

    @Override
    public void end(boolean interrupted) {
        turretSb.setTurretPower(0);
    }

    @Override
    public boolean isFinished() {
        if(finishOnSetpoint) {
            return Math.abs(turretSb.turretPid.getPositionError()) <= 5;
        } else {
            return false;
        }
    }

}
