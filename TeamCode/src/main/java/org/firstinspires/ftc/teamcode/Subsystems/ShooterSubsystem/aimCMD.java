package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MAX_FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MIN_FLYWHEEL_SPEED;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MIN_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.PASS_THROUGH_POINT_RADIUS_CLOSE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.PASS_THROUGH_POINT_RADIUS_FAR;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_ANGLE_CLOSE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_ANGLE_FAR;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_HEIGHT_CLOSE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_HEIGHT_FAR;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.flywheelOffSetMultiplier_CLOSE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.flywheelOffSetMultiplier_FAR;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.flywheelOffSet_CLOSE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.flywheelOffSet_FAR;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.goalX_CLOSE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.goalX_FAR;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.goalY_CLOSE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.goalY_FAR;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.hoodAdjustment;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.kTurretvel;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.minflywheelClose;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.minflywheelFar;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.velocityShooterDeadPoint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import java.util.function.BooleanSupplier;

public class aimCMD extends CommandBase {

    ShooterSubsystem shooterSb;

    double hoodAngle = 0;
    double flywheelSpeed = 0;

    int goalX = goalX_FAR;
    int goalY = goalY_FAR;
    double SCORE_HEIGHT = SCORE_HEIGHT_FAR; //inches
    double SCORE_ANGLE = SCORE_ANGLE_FAR; //inches

    double PASS_THROUGH_POINT_RADIUS = PASS_THROUGH_POINT_RADIUS_FAR; //inches

    double flywheelOffSetMultiplier = flywheelOffSetMultiplier_FAR;

    double flywheelOffSet = flywheelOffSet_FAR;

    BooleanSupplier isClose;
    boolean isShooting = false;

    boolean reAnguled = false;

    public aimCMD(ShooterSubsystem shooterSubsystem, BooleanSupplier isClose) {

        this.shooterSb = shooterSubsystem;

        this.isShooting = false;

        this.isClose = isClose;

        addRequirements(shooterSubsystem);
    }

    public aimCMD(ShooterSubsystem shooterSubsystem, boolean isShooting, BooleanSupplier isClose) {

        this.shooterSb = shooterSubsystem;

        this.isShooting = isShooting;

        this.isClose = isClose;

        addRequirements(shooterSubsystem);
    }

    public aimCMD(ShooterSubsystem shooterSubsystem, boolean isShooting, boolean isClose) {

        this.shooterSb = shooterSubsystem;

        this.isShooting = isShooting;

        this.isClose = ()-> isClose;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        Pose2d robotPose = new Pose2d(shooterSb.follower.getPose().getX(), shooterSb.follower.getPose().getY(), shooterSb.follower.getPose().getHeading());

        double heading = robotPose.getHeading();

// shooter is 2.5 inches behind robot center
        double offsetX = -2.5;

// rotate offset into global frame
        double shooterX = robotPose.getX() + offsetX * Math.cos(heading);
        double shooterY = robotPose.getY() + offsetX * Math.sin(heading);

        Pose2d shooterPose = new Pose2d(shooterX, shooterY, heading);

        if (shooterPose.getY() < 50) {
            goalX = goalX_FAR;
            goalY = goalY_FAR;
            SCORE_HEIGHT = SCORE_HEIGHT_FAR; //inches
            SCORE_ANGLE = SCORE_ANGLE_FAR; //inches

            PASS_THROUGH_POINT_RADIUS = PASS_THROUGH_POINT_RADIUS_FAR; //inches

            flywheelOffSetMultiplier = flywheelOffSetMultiplier_FAR;

            flywheelOffSet = flywheelOffSet_FAR;

        } else {
            goalX = goalX_CLOSE;
            goalY = goalY_CLOSE;
            SCORE_HEIGHT = SCORE_HEIGHT_CLOSE; //inches
            SCORE_ANGLE = SCORE_ANGLE_CLOSE; //inches

            PASS_THROUGH_POINT_RADIUS = PASS_THROUGH_POINT_RADIUS_CLOSE;

            flywheelOffSetMultiplier = flywheelOffSetMultiplier_CLOSE;

            flywheelOffSet = flywheelOffSet_CLOSE;
        }

        Translation2d goalPosition = new Translation2d(goalX, goalY);

        Translation2d robotToGoal =
                goalPosition.minus(shooterPose.getTranslation());

        Vector2d robotToGoalVector =
                new Vector2d(robotToGoal.getX(), robotToGoal.getY());

        double g = 32.174 * 12;
        double x = robotToGoalVector.magnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

//calculate initial launch components
        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(MAX_HOOD_ANGLE), Math.toRadians(MIN_HOOD_ANGLE));

        double denom = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("denominador", (g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y))));

        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        Vector robotVelocity = shooterSb.follower.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.angle();

        double parralelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

//velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parralelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

//recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr),
                Math.toRadians(MAX_HOOD_ANGLE), Math.toRadians(MIN_HOOD_ANGLE));

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2)
                * (ndr * Math.tan(hoodAngle) - y)));

        double flywheelTarget;

        if (isClose.getAsBoolean()) {
            flywheelTarget = MathFunctions.clamp(getFlywheelTicksFromVelocitya(flywheelSpeed), minflywheelClose, MAX_FLYWHEEL_SPEED);

        } else {
            flywheelTarget = MathFunctions.clamp(getFlywheelTicksFromVelocitya(flywheelSpeed), minflywheelFar, MAX_FLYWHEEL_SPEED);

        }
//update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr) * kTurretvel;
        double turretAngle = Math.toDegrees(shooterPose.getHeading() - robotToGoalVector.angle() + turretVelCompOffset);

//double turretAngle = shooterSb.turretToGoalAngle - Math.toDegrees(turretVelCompOffset);

        double finalHoodAngle;

        if (isShooting && Math.abs(shooterSb.shooterError) > velocityShooterDeadPoint && !reAnguled) {
            hoodAngle += hoodAdjustment;
            reAnguled = true;
        } else {
            reAnguled = false;
            finalHoodAngle = hoodAngle;
        }

        shooterSb.setShooterTarget(flywheelTarget);
        shooterSb.setHoodPose(MathFunctions.clamp(Math.toDegrees(hoodAngle), MAX_HOOD_ANGLE + 0.001, MIN_HOOD_ANGLE - 0.001));
        shooterSb.setTurretTarget(turretAngle);
    }
   /* @Override
    public void execute() {

        Pose robotPose = shooterSb.follower.getPose();

        double heading = robotPose.getHeading();

// shooter is 2.5 inches behind robot center
        double offsetX = -2.5;
        double offsetY = 0;

// rotate offset into global frame
        double shooterX = robotPose.getX() + offsetX * Math.cos(heading) - offsetY * Math.sin(heading);
        double shooterY = robotPose.getY() + offsetX * Math.sin(heading) + offsetY * Math.cos(heading);

        Pose2d shooterPose = new Pose2d(shooterX, shooterY, heading);

        if (shooterPose.getY() < 50){
            goalX = goalX_FAR;
            goalY = goalY_FAR;
            SCORE_HEIGHT = SCORE_HEIGHT_FAR; //inches
            SCORE_ANGLE = SCORE_ANGLE_FAR; //inches

            PASS_THROUGH_POINT_RADIUS = PASS_THROUGH_POINT_RADIUS_FAR; //inches

            flywheelOffSetMultiplier = flywheelOffSetMultiplier_FAR;

            flywheelOffSet = flywheelOffSet_FAR;

        }else {
            goalX = goalX_CLOSE;
            goalY = goalY_CLOSE;
            SCORE_HEIGHT = SCORE_HEIGHT_CLOSE; //inches
            SCORE_ANGLE = SCORE_ANGLE_CLOSE; //inches

            PASS_THROUGH_POINT_RADIUS = PASS_THROUGH_POINT_RADIUS_CLOSE;

            flywheelOffSetMultiplier = flywheelOffSetMultiplier_CLOSE;

            flywheelOffSet = flywheelOffSet_CLOSE;
        }

        Translation2d goalPosition = new Translation2d(goalX, goalY);

        Translation2d robotToGoal =
                goalPosition.minus(shooterPose.getTranslation());

        Vector2d robotToGoalVector =
                new Vector2d(robotToGoal.getX(), robotToGoal.getY());

        double g = 32.174 * 12;
        double x = robotToGoalVector.magnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        //calculate initial launch components
        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(MAX_HOOD_ANGLE), Math.toRadians(MIN_HOOD_ANGLE));

        double denom = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);

        flywheelSpeed =  Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        Vector robotVelocity = shooterSb.follower.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.angle();

        double parralelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parralelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr =  nvr * time;

        //recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr),
                Math.toRadians(MAX_HOOD_ANGLE), Math.toRadians(MIN_HOOD_ANGLE));
        
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2)
                * (ndr * Math.tan(hoodAngle) - y)));

        double flywheelTarget;

        if (isClose.getAsBoolean()){
            flywheelTarget = MathFunctions.clamp(getFlywheelTicksFromVelocitya(flywheelSpeed), minflywheelClose, MAX_FLYWHEEL_SPEED);

        }else {
            flywheelTarget = MathFunctions.clamp(getFlywheelTicksFromVelocitya(flywheelSpeed), minflywheelFar, MAX_FLYWHEEL_SPEED);

        }
        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = Math.toDegrees(shooterPose.getHeading() - robotToGoalVector.angle() + (turretVelCompOffset * kTurretvel));

        //double turretAngle = shooterSb.turretToGoalAngle - (Math.toDegrees(turretVelCompOffset) * kTurretvel);

        double finalHoodAngle;

        if (isShooting && Math.abs(shooterSb.shooterError.getAsDouble()) > velocityShooterDeadPoint && !reAnguled){
            hoodAngle += hoodAdjustment;
            reAnguled = true;
        }else {
            reAnguled = false;
            finalHoodAngle = hoodAngle;
        }

        shooterSb.setShooterTarget(flywheelTarget);
        shooterSb.setHoodPose(MathFunctions.clamp(Math.toDegrees(hoodAngle), MAX_HOOD_ANGLE + 0.001, MIN_HOOD_ANGLE - 0.001));
        shooterSb.setTurretTarget(turretAngle);

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("flywheelSpeed", flywheelSpeed);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("hoodPose", hoodAngle);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("turretTargetAngle", turretAngle);

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("isClose", isClose);

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("shooterPose", shooterPose);

    }

    */

    public double getFlywheelTicksFromVelocitya(double velocity) {
        double wheelRadius = 1.889; // inches (CHANGE THIS)
        double ticksPerRev = 28;  // goBILDA encoder (adjust if geared)

        double ticksPerSecond = (velocity / wheelRadius) * (ticksPerRev / (2 * Math.PI));

        return MathFunctions.clamp(
                Math.pow((ticksPerSecond + flywheelOffSet), flywheelOffSetMultiplier),
                MIN_FLYWHEEL_SPEED,
                MAX_FLYWHEEL_SPEED
        );
    }

}