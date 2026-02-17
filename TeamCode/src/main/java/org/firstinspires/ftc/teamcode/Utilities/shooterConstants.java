package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

@Config
public class shooterConstants {

    public static Pose GOAL_POSE_RED = new Pose(138, 138);
    public static Pose GOAL_POSE_BLUE = GOAL_POSE_RED.mirror();

    public static double SCORE_HEIGHT = 26; //inches
    public static double SCORE_ANGLE = Math.toRadians(-30); //inches

    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    public static double MAX_HOOD_ANGLE = 50;

    public static double MIN_HOOD_ANGLE = 20;

    public static double MAX_FLYWHEEL_SPEED = 2200;

    public static double MIN_FLYWHEEL_SPEED = 0;

    public static double flywheelOffSet = 0;


    public static double getFlywheelTicksFromVelocity(double velocity){
        return MathFunctions.clamp(94.501 * velocity / 12 - 187.96 +  flywheelOffSet,
                MIN_FLYWHEEL_SPEED, MAX_FLYWHEEL_SPEED);

    }

    public static double gethoodTicksFromDegrees(double degrees){
        return 0.0226 * degrees - 0.7443;


    }

    public Vector calculateShotVectorAndUpdateTurret(double robotHeading){
        double g = 32.174 * 12;

        return null;
    }
}
