package org.firstinspires.ftc.teamcode.Subsystems.Shooter;


import com.seattlesolvers.solverslib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class shooterToVelCMD extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;

    DoubleSupplier targetVel;


    public shooterToVelCMD(ShooterSubsystem shooterSb, DoubleSupplier targetVel) {
        shooterSubsystem = shooterSb;

        this.targetVel = targetVel;
        addRequirements(shooterSubsystem);
    }

    public shooterToVelCMD(ShooterSubsystem shooterSb, double targetVel) {
        shooterSubsystem = shooterSb;

        this.targetVel = () -> targetVel;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.shooterTarget = targetVel.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.shooterTarget = ShooterSubsystem.standarShooterVel;

    }

}
