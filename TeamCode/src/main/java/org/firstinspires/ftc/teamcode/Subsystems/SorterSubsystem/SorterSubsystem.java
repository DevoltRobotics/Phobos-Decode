package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SorterSubsystem extends SubsystemBase {

    Servo blockerR;
    Servo blockerL;

    Servo blockerH;

    RevColorSensorV3 colorR;
    RevColorSensorV3 colorL;

    public double greenInR;
    public double greenInL;

    public double blueInR;
    public double blueInL;

    public double power = 0;

    public Telemetry telemetry;

    //CONSTANTS

    public static double blockersUp = -0.2;

    public static double blockerHHidePos = 0.45;
    public static double blockerHFreePos = 0.75;

    public SorterSubsystem(HardwareMap hMap, Telemetry telemetry) {
        blockerR = hMap.servo.get("blcR");
        blockerL = hMap.servo.get("blcL");
        blockerH = hMap.servo.get("blcH");
        colorR = hMap.get(RevColorSensorV3.class, "colorR");
        colorL = hMap.get(RevColorSensorV3.class, "colorL");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        greenInR = colorR.green();
        greenInL = colorL.green();
        blueInR = colorR.blue();
        blueInL = colorL.blue();

        telemetry.addData("RightGreen", greenInR);
        telemetry.addData("LeftGreen", greenInL);
        telemetry.addData("RightBlue", blueInR);
        telemetry.addData("LeftBlue", blueInL);
    }
}
