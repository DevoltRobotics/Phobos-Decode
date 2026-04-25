package org.firstinspires.ftc.teamcode.Utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class shooterConstants {

    //POSES
    public static int goalX_CLOSE = 144;
    public static int goalY_CLOSE = 138;
    public static double SCORE_HEIGHT_CLOSE = 26; //inches 30
    public static double SCORE_ANGLE_CLOSE = Math.toRadians(-30); //inches

    public static double PASS_THROUGH_POINT_RADIUS_CLOSE = 5; //inches

    ////

    public static int goalX_FAR = 136;
    public static int goalY_FAR = 138;
    public static double SCORE_HEIGHT_FAR = 16; //inches
    public static double SCORE_ANGLE_FAR = -0.65;// Math.toRadians(-30); //inches
    public static double PASS_THROUGH_POINT_RADIUS_FAR = 7; //inches

    //HOOD
    public static double MAX_HOOD_ANGLE = 42;

    public static double MIN_HOOD_ANGLE = 67;

    public static double servoHoodRatio = (double) 259 / 25;

    public static double hoodAdjustment = 0.12; //16

    public static double gethoodTicksFromDegrees(double degrees){
        double range = MIN_HOOD_ANGLE - MAX_HOOD_ANGLE; //25
        double servo_Min = (range * servoHoodRatio) / 360; //0.719

        double normalizedAngle = MathFunctions.clamp(degrees, MAX_HOOD_ANGLE, MIN_HOOD_ANGLE);

        return MathFunctions.clamp((servo_Min + (( (MAX_HOOD_ANGLE - normalizedAngle) * servoHoodRatio) / 360)), 0.001, 0.999);

    }

    //FLYWHEEL

    public static double shooterkV = 0.00057;
    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(0.02, 0.0, 0.0, 0);

    // ===============================
// Convert flywheel linear velocity (in/s) → ticks/sec
// ===============================

// Clamp to motor capability


    public static double MAX_FLYWHEEL_SPEED = 2000;

    public static double MIN_FLYWHEEL_SPEED = 800;

    public static double minflywheelClose = 800;
    public static double minflywheelFar = 1500;

    public static double flywheelOffSet_CLOSE = 175;

    public static double flywheelOffSetMultiplier_CLOSE = 1.093;

    public static double flywheelOffSet_FAR = 115;

    public static double flywheelOffSetMultiplier_FAR = 1.12;

    public static double velocityShooterDeadPoint = 60;

    public static double getFlywheelTicksFromVelocity(double velocity) {
        double wheelRadius = 1.889; // inches (CHANGE THIS)
        double ticksPerRev = 28;  // goBILDA encoder (adjust if geared)

        double ticksPerSecond = (velocity / wheelRadius) * (ticksPerRev / (2 * Math.PI));

        return MathFunctions.clamp(
                Math.pow((ticksPerSecond + flywheelOffSet_CLOSE),  flywheelOffSetMultiplier_CLOSE),
                MIN_FLYWHEEL_SPEED,
                MAX_FLYWHEEL_SPEED
        );
    }

    //TURRET
    public static double kTurretvel = 0.75;

    public static PIDFCoefficients principalTurretCoeffs = new PIDFCoefficients(0.04, 0.0, 0.002, 0.001); //f = 0.001

    public static PIDFCoefficients secondaryTurretCoeffs = new PIDFCoefficients(0.07, 0.0, 0.0008, 0.0004);

    public static double kG = 0.02; // start small, tune later
    public static double thetaOffset = Math.toRadians(120);

    public static double turretPidSwitch = 3.5; // start SMALL

    public static double minimunPower = 0.042;

    public static double capstanRatio = 0.315; //0.331

    public static double ticktsToDegrees = (double) 360 / 8192;

    public static int upperLimit = 102;

    public static int lowerLimit = -110;

    public static int manualIncrement = 5;

}
