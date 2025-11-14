package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.seattlesolvers.solverslib.command.CommandBase;

public class turretToPosCMD extends CommandBase {

    private final TurretSubsystem turretSubsystem;

    double basketTarget;

    public turretToPosCMD(TurretSubsystem turretSb, double targetPos) {
        turretSubsystem = turretSb;

        this.basketTarget = targetPos;
        addRequirements(turretSubsystem);
    }


    @Override
    public void initialize() {
        turretSubsystem.isTurretManual = false;

    }

    @Override
    public void execute() {
        turretSubsystem.isTurretManual = false;
        turretSubsystem.turretTarget = basketTarget;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
