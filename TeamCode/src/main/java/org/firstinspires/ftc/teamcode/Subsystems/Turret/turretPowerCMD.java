package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.seattlesolvers.solverslib.command.CommandBase;

public class turretPowerCMD extends CommandBase {

    TurretSubsystem turretsb;

    double power;

    public turretPowerCMD(TurretSubsystem subsystem, double power) {
        this.turretsb = subsystem;
        this.power = power;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        //turretsb.setTurretPower(power);
    }

    @Override
    public void end(boolean interrupted) {
       // turretsb.setTurretPower(0);
    }
}
