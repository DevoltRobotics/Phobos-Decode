package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MIN_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.PASS_THROUGH_POINT_RADIUS;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_HEIGHT;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

public class aimCMD extends CommandBase {

    ShutSubsystem shooterSb;

    double hoodAngle = 0;
    double flywheelSpeed = 0;
    double robotHeading = 0;

    public aimCMD(ShutSubsystem shootertSubsystem) {

        this.shooterSb = shootertSubsystem;

        addRequirements(shootertSubsystem);
    }

    @Override
    public void execute() {
        Pose2d robotPose = new Pose2d(shooterSb.follower.getPose().getX(), shooterSb.follower.getPose().getY(), shooterSb.follower.getPose().getHeading());

        Translation2d goalPosition = new Translation2d(shooterSb.goalX, shooterSb.goalY);

        Translation2d robotToGoal =
                goalPosition.minus(robotPose.getTranslation());

        Vector robotToGoalVector =
                new Vector(robotToGoal.getX(), robotToGoal.getY());

        Vector robotVelocity = shooterSb.follower.getVelocity();

        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        //calculate initial launch components
        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.atan(a)), MAX_HOOD_ANGLE, MIN_HOOD_ANGLE);

        flywheelSpeed =  Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parralelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parralelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr =  nvr * time;

        //recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), MAX_HOOD_ANGLE, MIN_HOOD_ANGLE);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle))));

        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() * turretVelCompOffset);


        shooterSb.setShooterTarget(flywheelSpeed);
        shooterSb.setTurretTarget(turretAngle);
        shooterSb.setHoodPose(hoodAngle);

    }
}