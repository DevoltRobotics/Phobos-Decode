package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Utilities.Aliance;

@Config
public class turretToBasketCMD extends CommandBase {

    private final TurretSubsystem turretSb;

    private final Aliance aliance;
    private final Follower follower;
    private final ElapsedTime deadTimer;

    double goalX = 0;
    double goalY = 140;


    public turretToBasketCMD(TurretSubsystem turretSb, Aliance alliance, Follower follower) {
        this.turretSb = turretSb;
        this.aliance = alliance;

        this.follower = follower;

        deadTimer = new ElapsedTime();

        addRequirements(turretSb);
    }

    @Override
    public void initialize() {
        goalX = aliance.equals(Aliance.RED) ? 140 : 4;
    }

    @Override
    public void execute() {

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = Math.toDegrees(turretSb.follower.getPose().getHeading());

        double angleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

        double turretTargetRelative = wrapAngle(robotHeading - angleToGoal);

        turretSb.setTarget((int) turretTargetRelative);
    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}



