package org.firstinspires.ftc.teamcode.Utilities;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.opmodecontrol.OpModeControlPluginConfig;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class StaticConstants extends OpModeControlPluginConfig {

    public static PIDFCoefficients liftingCoeffs = new PIDFCoefficients(0.0, 0.0, 0.0, 0);

    public static double shooterkV = 0.000468;
    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(0.1, 0.0, 0.0, 0);

    //TURRET

    public static double turretRatio = (double) 58 / 185;
    public static double turretRatioBtoL = (double) 185 / 58;

    public static int maxTurretTurnDegrees = 100;

    public static double blueBasketAngle = 45;
    public static double redBasketAngle = 135;

    public static int maxTicksPerSecShooter = (int)(28 * 77.3);

    public static int limelightTaRatio = 100;

    public static double shoootVsint1 = 28.5, shoootVsint2 = 36, shoootVsint3 = 80, shoootVsint4 = 153, shoootVsint5 = 270;;
    public static double shoootVsout1 = 1450, shoootVsout2 = 1350, shoootVsout3 = 1250, shoootVsout4 = 1100, shoootVsout5 = 1000;

    public static double shoootPosint1 = 0, shoootPosint2 = 0, shoootPosint3 = 0, shoootPosint4 = 0;
    public static double shoootPosout1 = 0, shoootPosout2 = 0, shoootPosout3 = 0, shoootPosout4 = 0;

    //SERVOS


}
