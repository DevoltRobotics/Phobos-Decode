package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shooterCoeffs;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.StaticConstants;

public class ShooterSubsystem extends SubsystemBase {

    DcMotorEx shooterMDown;
    DcMotorEx shooterMUp;

    PIDFController shooterController = new PIDFController(shooterCoeffs);

    Telemetry telemetry;

    public InterpLUT vsFunc = new InterpLUT();

    public double targetdistance = shoootVsint2;

    public boolean isBlueAliance;

    double shooterTarget;

    //CONSTANTS

    public static int standarShooterVel = 900;

    public ShooterSubsystem(HardwareMap hMap, Telemetry telemetry, boolean isBlueAliance) {

        shooterMDown = hMap.get(DcMotorEx.class, "shdown");
        shooterMUp = hMap.get(DcMotorEx.class, "shup");

        shooterMDown.setDirection(DcMotorSimple.Direction.REVERSE);

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
        double shooterTargetPwr = (StaticConstants.shooterkV * shooterTarget) + shooterController.calculate(motorVel);

        shooterMUp.setPower(shooterTargetPwr);
        shooterMDown.setPower(shooterTargetPwr);

        /*telemetry.addData("upsVelocity", motorVel);
        telemetry.addData("shooterTarget", shooterTarget);
        telemetry.addData("vError", velocityError);
        telemetry.addData("ta", targetdistance);

         */
    }
}
