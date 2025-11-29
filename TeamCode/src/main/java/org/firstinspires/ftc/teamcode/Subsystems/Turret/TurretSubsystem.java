package org.firstinspires.ftc.teamcode.Subsystems.Turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TurretSubsystem extends SubsystemBase {

    public CRServo turretM;
    public AnalogInput turretE;

    public static PIDFCoefficients principalTurretCoeffs = new PIDFCoefficients(0.02, 0.0, 0.00045, 0);
    public PIDFController principalTurretController = new PIDFController(principalTurretCoeffs);

    public static PIDFCoefficients llPidCoeffs = new PIDFCoefficients(0.043, 0.0, 0.0001, 0);
    public PIDFController llPidf = new PIDFController(llPidCoeffs);

    public static double turretRatio = (double) 58 / 185;
    public static double turretRatioBtoL = (double) 185 / 58;

    public boolean realIsManual;

    public static double furtherCorrection = 2.5;

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

    public boolean isAlianceBlue;

    Telemetry telemetry;

    double newTurretTarget = 0;

    double turretAbsolutepos = 0;

    public TurretSubsystem(HardwareMap hMap, Telemetry telemetry) {
        turretM = hMap.get(CRServo.class, "trt");
        turretE = hMap.get(AnalogInput.class, "trtE");

        this.telemetry = telemetry;

    }

    @Override
    public void periodic() {
        principalTurretController.setCoefficients(principalTurretCoeffs);

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

        turretPRelative -= (deltaPos * turretRatio);

        turretTarget = Range.clip(turretTarget, -maxTurretTurnDegrees, maxTurretTurnDegrees);

        error = turretTarget - turretPRelative;

        principalTurretController.setSetPoint(turretTarget);

        pidPower = principalTurretController.calculate(turretPRelative);


        double targetPower = Range.clip(pidPower, -1, 1);

        turretM.setPower(targetPower);

        lastTurretP = turretP;

        FtcDashboard.getInstance().getTelemetry().addData("TurretPRel", turretPRelative);
        FtcDashboard.getInstance().getTelemetry().addData("TurretTarget", turretTarget);
        FtcDashboard.getInstance().getTelemetry().addData("TurretError", error);

        telemetry.addData("realIsManual", realIsManual);
        telemetry.addData("turretAbsolutePos", turretAbsolutepos);

        llPidf.setCoefficients(llPidCoeffs);
        llPidf.setSetPoint(0);
    }


}
