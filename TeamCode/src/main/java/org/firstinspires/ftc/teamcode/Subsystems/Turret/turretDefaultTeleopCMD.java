package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.manualIncrement;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MAX_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.MIN_HOOD_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.PASS_THROUGH_POINT_RADIUS;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_ANGLE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.SCORE_HEIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.function.BooleanSupplier;

public class turretDefaultTeleopCMD extends CommandBase {

    private final TurretSubsystem turretSb;

    BooleanSupplier isManual;

    BooleanSupplier isShooting;

    Gamepad gamepad;

    public turretDefaultTeleopCMD(TurretSubsystem turretSb, BooleanSupplier isManual, Gamepad gamepad, BooleanSupplier isShooting) {
        this.turretSb = turretSb;

        this.gamepad = gamepad;

        this.isManual = isManual;


        this.isShooting = isShooting;

        addRequirements(turretSb);
    }

    @Override
    public void execute() {

        Pose2d robotPose = new Pose2d(turretSb.follower.getPose().getX(), turretSb.follower.getPose().getY(), turretSb.follower.getPose().getHeading());

        Translation2d goalPosition = new Translation2d(turretSb.goalX, turretSb.goalY);

        Translation2d robotToGoal =
                goalPosition.minus(robotPose.getTranslation());

        Vector robotToGoalVector =
                new Vector(robotToGoal.getX(), robotToGoal.getY());

        Vector robotVelocity = turretSb.follower.getVelocity();

        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        //calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.atan(a)), MAX_HOOD_ANGLE, MIN_HOOD_ANGLE);

        double flywheelSpeed =  Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parralelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parralelComponent;

        //recalculate launch components

        //update turret
        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);
        //double turretAngle = Math.toDegrees(robotPose.getHeading() - robotToGoalVector.getTheta() + turretVelCompOffset);

        double turretAngle = turretSb.turretToGoalAngle - Math.toDegrees(turretVelCompOffset);

        if (gamepad.right_bumper || gamepad.left_bumper) {
            turretSb.realIsManual = true;

        } else {
            turretSb.realIsManual = isManual.getAsBoolean();

        }

        if (turretSb.realIsManual) {
            if (gamepad.right_bumper) {
                turretSb.turretTarget += manualIncrement;

            } else if (gamepad.left_bumper) {
                turretSb.turretTarget -= manualIncrement;

            }
        }
        else {
            turretSb.setTarget(turretAngle);

        }
    }
}