package org.firstinspires.ftc.teamcode.Subsystems.Turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;

@Config
public class TurretSubsystem extends SubsystemBase {

    public CRServo turretS1;
    public CRServo turretS2;

    public AnalogInput turretE;

    public static PIDFCoefficients principalTurretCoeffs = new PIDFCoefficients(0.02, 0.0, 0.00045, 0);
    public PIDFController principalTurretController = new PIDFController(principalTurretCoeffs);

    //public static PIDFCoefficients llPidCoeffs = new PIDFCoefficients(0.043, 0.0, 0.0001, 0);
    //public PIDFController llPidf = new PIDFController(llPidCoeffs);

    public static PIDFCoefficients anglePidCoeffs = new PIDFCoefficients(0.043, 0.0, 0.0001, 0);
    public PIDFController anglePidController = new PIDFController(anglePidCoeffs);

    public static double gearRatio = 2;
    public static double turretRatio = (double) 58 / 185;
    public static double turretRatioBtoL = (double) 185 / 58;

    public boolean realIsManual;

    public static double furtherCorrectionAuto = 2;

    public static double furtherCorrection = 3;

    public static double manualIncrement = 2.5;

    public static int maxTurretTurnDegrees = 110;
    public double turretP = 0;
    public Double lastTurretP = null;

    public double deltaPos;

    public static double startTurretPos = 20;
    public double turretPRelative = startTurretPos;

    public double turretTarget = 0;

    public double error;

    public double pidPower;

    double turretAbsolutepos = 0;

    public Telemetry telemetry;

    public Follower follower;

    public Aliance aliance;

    public TurretSubsystem(HardwareMap hMap, Follower follower, Telemetry telemetry, Aliance aliance) {
        turretS1 = hMap.get(CRServo.class, "trt1");
        turretS2 = hMap.get(CRServo.class, "trt2");
        turretE = hMap.get(AnalogInput.class, "trtE");

        turretS2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
        this.aliance = aliance;

    }

    @Override
    public void periodic() {
        principalTurretController.setCoefficients(principalTurretCoeffs);

        anglePidController.setCoefficients(anglePidCoeffs);
        anglePidController.setSetPoint(0);

        turretP = (turretE.getVoltage() / 3.3) * 360;

        if (lastTurretP == null) {
            lastTurretP = turretP;
        }

        deltaPos = turretP - lastTurretP;

        if (Math.abs(deltaPos) < 0.05) {
            deltaPos = 0;
        }

        if (deltaPos > 180) {
            deltaPos -= 360;

        } else if (deltaPos < -180) {
            deltaPos += 360;
        }

        turretPRelative -= (deltaPos * turretRatio * gearRatio);

        turretTarget = Range.clip(turretTarget, -maxTurretTurnDegrees, maxTurretTurnDegrees);

        error = turretTarget - turretPRelative;

        principalTurretController.setSetPoint(turretTarget);

        pidPower = principalTurretController.calculate(turretPRelative);

        double targetPower = Range.clip(pidPower, -1, 1);

        turretS1.setPower(targetPower);
        turretS2.setPower(targetPower);

        lastTurretP = turretP;

        FtcDashboard.getInstance().getTelemetry().addData("TurretPRel", turretPRelative);
        FtcDashboard.getInstance().getTelemetry().addData("TurretTarget", turretTarget);
        FtcDashboard.getInstance().getTelemetry().addData("TurretError", error);

        telemetry.addData("realIsManual", realIsManual);
        telemetry.addData("turretAbsolutePos", turretAbsolutepos);

        double chassisHeading = follower.getPose().getHeading(); // degrees
        turretAbsolutepos = wrapAngle(chassisHeading - turretPRelative);


    }

    public void setTarget (int target){
        turretTarget = target;

    }

    public double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }


}
