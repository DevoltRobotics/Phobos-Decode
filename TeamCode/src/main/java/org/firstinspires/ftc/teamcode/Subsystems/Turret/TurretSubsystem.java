package org.firstinspires.ftc.teamcode.Subsystems.Turret;

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


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TurretSubsystem extends SubsystemBase {

    public CRServo turretM;
    public AnalogInput turretE;

    public static PIDFCoefficients turretCoeffs = new PIDFCoefficients(0.02, 0.0, 0.0003, 0);
    public PIDFController turretController = new PIDFController(turretCoeffs);

    public static PIDFCoefficients llPidCoeffs = new PIDFCoefficients(0.043, 0.0, 0.0001, 0);
    public PIDFController llPidf;

    public static PIDFCoefficients headingPidfCoeffs = new PIDFCoefficients(0.043, 0.0, 0.0001, 0);
    public PIDFController headingPidfController;

    public static double turretRatio = (double) 58 / 185;
    public static double turretRatioBtoL = (double) 185 / 58;

    public static int maxTurretTurnDegrees = 100;
    public double turretP = 0;
    public Double lastTurretP = null;

    public double turretPRelative = 24;

    public double turretTarget = 0;

    public static double manualIncrement = 3;

    public double error;

    public double turretPower;

    public double pidPower;

    public boolean isAlianceBlue;

    Telemetry telemetry;

    public InterpLUT turretvsFunc = new InterpLUT();

    public TurretSubsystem(HardwareMap hMap, Telemetry telemetry, boolean isAlianceBlue) {
        turretM = hMap.get(CRServo.class,"trt");
        turretE = hMap.get(AnalogInput.class, "trtE");

        llPidf = new PIDFController(llPidCoeffs);
        headingPidfController = new PIDFController(headingPidfCoeffs);

        this.telemetry = telemetry;
        this.isAlianceBlue = isAlianceBlue;

        turretvsFunc.add(shoootVsint1, shoootVsout1);
        turretvsFunc.add(shoootVsint2, shoootVsout2);
        turretvsFunc.add(shoootVsint3, shoootVsout3);
        turretvsFunc.add(shoootVsint4, shoootVsout4);
        turretvsFunc.add(shoootVsint5, shoootVsout5);
        turretvsFunc.createLUT();
    }

    @Override
    public void periodic() {
        turretController.setCoefficients(turretCoeffs);
        turretP = (turretE.getVoltage() / 3.3) * 360;

        if (lastTurretP == null) {
            lastTurretP = turretP;
        }

        double deltaPos = turretP - lastTurretP;

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

        pidPower = turretController.calculate(turretPRelative);

        error = turretTarget - turretPRelative;

        double targetPower = Range.clip(turretPower + pidPower, -0.85, 0.85);

        turretM.setPower(targetPower);
        turretController.setSetPoint(turretTarget);

        lastTurretP = turretP;

        FtcDashboard.getInstance().getTelemetry().addData("TurretPRel", turretPRelative);
        FtcDashboard.getInstance().getTelemetry().addData("TurretTarget", turretTarget);
        FtcDashboard.getInstance().getTelemetry().addData("TurretError", error);


        llPidf.setCoefficients(llPidCoeffs);
        llPidf.setSetPoint(0);

        headingPidfController.setCoefficients(headingPidfCoeffs);
        headingPidfController.setSetPoint(0);

    }
}
