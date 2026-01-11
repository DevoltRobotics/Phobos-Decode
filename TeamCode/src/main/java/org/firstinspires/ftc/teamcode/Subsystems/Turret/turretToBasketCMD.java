package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;

@Config
public class turretToBasketCMD extends CommandBase {

    TurretSubsystem turretSb;

    public turretToBasketCMD(TurretSubsystem turretSb) {

        addRequirements(turretSb);
    }

    @Override
    public void execute() {

        turretSb.turretTarget = turretSb.getTurretToGoalAngle();

        super.execute();
    }

}



