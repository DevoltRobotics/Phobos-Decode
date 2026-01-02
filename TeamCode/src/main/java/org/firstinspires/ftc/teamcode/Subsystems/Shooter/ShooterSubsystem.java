package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint6;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout0;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout6;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

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

    public boolean isBlueAliance;

    public double shooterTarget = 0;

    //CONSTANTS

    public static int standarShooterVel = 0;

    public ShooterSubsystem(HardwareMap hMap, Telemetry telemetry) {

        shooterMDown = hMap.get(DcMotorEx.class, "shdown");
        shooterMUp = hMap.get(DcMotorEx.class, "shup");

        shooterMUp.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        shooterController.setCoefficients(shooterCoeffs);

        double motorVel = shooterMUp.getVelocity();

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
