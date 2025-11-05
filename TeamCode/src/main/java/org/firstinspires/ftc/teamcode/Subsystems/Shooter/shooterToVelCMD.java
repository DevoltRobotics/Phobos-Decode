package org.firstinspires.ftc.teamcode.Subsystems.Shooter;


import com.seattlesolvers.solverslib.command.CommandBase;

public class shooterToVelCMD extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;

    double targetVel;

    public shooterToVelCMD(ShooterSubsystem shooterSb, double targetVel) {
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
