package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint0;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout0;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout5;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;
import org.firstinspires.ftc.teamcode.Utilities.StaticConstants;

@Config
public class ShooterSubsystem extends SubsystemBase {

    DcMotorEx shooterMDown;
    DcMotorEx shooterMUp;

    public static double shooterkV = 0.000565;
    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(0.01, 0.0, 0.0, 0);
    PIDFController shooterController = new PIDFController(shooterCoeffs);

    Telemetry telemetry;

    public InterpLUT vsFunc = new InterpLUT();

    public boolean isBlueAliance;

    double shooterTarget = 0;

    //CONSTANTS

    public static int standarShooterVel = 0;

    public ShooterSubsystem(HardwareMap hMap, Telemetry telemetry, boolean isBlueAliance) {

        shooterMDown = hMap.get(DcMotorEx.class, "shdown");
        shooterMUp = hMap.get(DcMotorEx.class, "shup");

        shooterMUp.setDirection(DcMotorSimple.Direction.REVERSE);


        vsFunc.add(shoootVsint0, shoootVsout0);
        vsFunc.add(shoootVsint1, shoootVsout1);
        vsFunc.add(shoootVsint2, shoootVsout2);
        vsFunc.add(shoootVsint3, shoootVsout3);
        vsFunc.add(shoootVsint4, shoootVsout4);
        vsFunc.add(shoootVsint5, shoootVsout5);
//generating final equation
        vsFunc.createLUT();

        this.isBlueAliance = isBlueAliance;

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        double motorVel = shooterMUp.getVelocity();

        shooterController.setCoefficients(shooterCoeffs);

        shooterController.setSetPoint(shooterTarget);

        double velocityError = shooterTarget - motorVel;
        double shooterTargetPwr = (shooterkV * shooterTarget) + shooterController.calculate(motorVel);

        shooterMUp.setPower(shooterTargetPwr);
        shooterMDown.setPower(shooterTargetPwr);

        /*telemetry.addData("upsVelocity", motorVel);
        telemetry.addData("shooterTarget", shooterTarget);
        telemetry.addData("vError", velocityError);
        telemetry.addData("ta", targetdistance);

         */
    }
}
