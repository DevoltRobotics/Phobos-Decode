package org.firstinspires.ftc.teamcode.Utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Configurable
public class shooterConstants {

    //POSES
    public static Pose GOAL_POSE_RED = new Pose(138, 138);
    public static Pose GOAL_POSE_BLUE = GOAL_POSE_RED.mirror();
    public static double SCORE_HEIGHT = 27; //inches
    public static double SCORE_ANGLE = Math.toRadians(-30); //inches

    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    //HOOD
    public static double MAX_HOOD_ANGLE = 42;

    public static double MIN_HOOD_ANGLE = 67;

    public static double servoHoodRatio = (double) 259 / 25;

    public static double gethoodTicksFromDegrees(double degrees){
        double range = MIN_HOOD_ANGLE - MAX_HOOD_ANGLE; //25
        double servo_Min = (range * servoHoodRatio) / 360; //0.719

        double normalizedAngle = MathFunctions.clamp(degrees, MAX_HOOD_ANGLE, MIN_HOOD_ANGLE);

        return servo_Min + (( (MAX_HOOD_ANGLE - normalizedAngle) * servoHoodRatio) / 360);

    }

    //FLYWHEEL

    public static double shooterkV = 0.000525;
    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(0.01, 0.0, 0.0, 0);

    // ===============================
// Convert flywheel linear velocity (in/s) → ticks/sec
// ===============================

    public static double FLYWHEEL_RADIUS = 1.89;
    public static double TICKS_PER_REV = 28.0;
    public static double MAX_TICKS_PER_SEC = 1600.0; // 6000 RPM motor

    /*public static double ticksFromSpeedFlyWheel (double flywheelSpeed){
        return Math.min(flywheelSpeed * (TICKS_PER_REV / (2.0 * Math.PI * FLYWHEEL_RADIUS)), MAX_TICKS_PER_SEC);
    }



     */
// Clamp to motor capability


    public static double MAX_FLYWHEEL_SPEED = 1700;

    public static double MIN_FLYWHEEL_SPEED = 1070;

    public static double flywheelOffSet = 0;

    public static double getFlywheelTicksFromVelocity(double velocity) {
        double wheelRadius = 1.889; // inches (CHANGE THIS)
        double ticksPerRev = 28;  // goBILDA encoder (adjust if geared)

        double ticksPerSecond = (velocity / wheelRadius) * (ticksPerRev / (2 * Math.PI));

        return MathFunctions.clamp(
                ticksPerSecond + flywheelOffSet,
                MIN_FLYWHEEL_SPEED,
                MAX_FLYWHEEL_SPEED
        );
    }

    //TURRET

    public static PIDFCoefficients principalTurretCoeffs = new PIDFCoefficients(0.02, 0.0, 0.0015, 0.00049);

    public static PIDFCoefficients secondaryTurretCoeffs = new PIDFCoefficients(0.035, 0.0, 0.0011, 0.00049);

    public static double turretPidSwitch = 10; // start SMALL

    public static double minimunPower = 0.03;

    public static double capstanRatio = 0.331; //0.331

    public static double ticktsToDegrees = (double) 360 / 537.7;

    public static int upperLimit = 110;

    public static int lowerLimit = -110;

    public static int manualIncrement = 5;

}
