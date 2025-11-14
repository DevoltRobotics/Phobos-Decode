package org.firstinspires.ftc.teamcode.Subsystems.Shooter;


import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class shooterToVelAutonomousCMD extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;

    double targetVel;

    public shooterToVelAutonomousCMD(ShooterSubsystem shooterSb, double targetVel) {
        shooterSubsystem = shooterSb;

        this.targetVel = targetVel;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.shooterTarget = targetVel;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
