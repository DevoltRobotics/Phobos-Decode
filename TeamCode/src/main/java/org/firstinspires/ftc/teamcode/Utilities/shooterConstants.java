package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.opencv.core.Mat;

@Config
public class shooterConstants {

    //POSES
    public static Pose GOAL_POSE_RED = new Pose(138, 138);
    public static Pose GOAL_POSE_BLUE = GOAL_POSE_RED.mirror();

    public static double SCORE_HEIGHT = 26; //inches
    public static double SCORE_ANGLE = Math.toRadians(-30); //inches

    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    //HOOD
    public static double MAX_HOOD_ANGLE = 40;

    public static double MIN_HOOD_ANGLE = 70;

    public static double hoodRatio = (double) 25 / 259;


    public static double gethoodTicksFromDegrees(double degrees){
        return (hoodRatio * degrees);


    }

    //FLYWHEEL
    public static double MAX_FLYWHEEL_SPEED = 2200;

    public static double MIN_FLYWHEEL_SPEED = 0;

    public static double flywheelOffSet = 0;


    public static double getFlywheelTicksFromVelocity(double velocity){
        return MathFunctions.clamp(94.501 * velocity / 12 - 187.96 +  flywheelOffSet,
                MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED);

    }

    //TURRET

    public static PIDFCoefficients principalTurretCoeffs = new PIDFCoefficients(0.013, 0.0, 0.0014, 0.00);

    public static double MinimumEnc = 0.07;

    public static double capstanRatio = (double) 58 / 182;

    public static double ticktsToDegrees = (double) 360 / 8192;

    public static int upperLimit = 110;

    public static int lowerLimit = -110;

    public static double turretPRelative = 0;

    public static double ffValue = -0.09;

    public Vector calculateShotVectorAndUpdateTurret(double robotHeading, Vector robotToGoalVector, Vector robotVelocity){
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

        return null;
    }
}
