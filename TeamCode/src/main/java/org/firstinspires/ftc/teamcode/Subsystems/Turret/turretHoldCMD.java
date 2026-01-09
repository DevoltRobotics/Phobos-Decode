package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Utilities.Aliance;

@Config
public class turretHoldCMD extends turretToPosCMD {

    public turretHoldCMD(TurretSubsystem turretSb) {
        super(turretSb, 0d, false);
        addRequirements(turretSb);
    }

    @Override
    public void execute() {

        targetPos = turretSb.getCurrentPosition();
        super.execute();
    }

}



