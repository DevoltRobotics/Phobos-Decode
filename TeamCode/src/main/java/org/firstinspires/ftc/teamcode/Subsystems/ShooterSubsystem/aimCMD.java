package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MIN_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.PASS_THROUGH_POINT_RADIUS;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_HEIGHT;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.getFlywheelTicksFromVelocity;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

public class aimCMD extends CommandBase {

    ShooterSubsystem shooterSb;

    double hoodAngle = 0;
    double flywheelSpeed = 0;

    public aimCMD(ShooterSubsystem shootertSubsystem) {

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
        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(MAX_HOOD_ANGLE), Math.toRadians(MIN_HOOD_ANGLE));

        double denom = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("denom", denom);

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
        hoodAngle = Math.atan(vz / nvr);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        //double turretAngle = Math.toDegrees(robotPose.getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset);

        double turretAngle = shooterSb.turretToGoalAngle - Math.toDegrees(turretVelCompOffset);

        shooterSb.setShooterTarget(getFlywheelTicksFromVelocity(flywheelSpeed));
        shooterSb.setHoodPose(MathFunctions.clamp(Math.toDegrees(hoodAngle), MAX_HOOD_ANGLE, MIN_HOOD_ANGLE));
        shooterSb.setTurretTarget(turretAngle);

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("flywheelSpeed", flywheelSpeed);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("hoodPose", hoodAngle);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("turretTarget", turretAngle);

    }
}