package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SorterSubsystem extends SubsystemBase {

    Servo blockerR;
    Servo blockerL;

    Servo blockerH;

    Servo ramp;

    public Telemetry telemetry;

    //CONSTANTS

    public static double upRampPos = 0.8;
    public static double downRampPos = 0.3;

    public static double blockersUp = -0.35;

    public static double blockerHHidePos = 0.53;
    public static double blockerHFreePos = 0.75;

    public static int waitAimTimer = 600;
    public static int artifacToArtifactTimer = 1300;

    enum BlockersStatus {
        closed,
        oneOpen,
        twoOpen

    }

    BlockersStatus blockersStatus = null;

    public boolean isShooting = false;
    public SorterSubsystem(HardwareMap hMap, Telemetry telemetry) {
        blockerR = hMap.servo.get("blcR");
        blockerL = hMap.servo.get("blcL");
        blockerH = hMap.servo.get("blcH");

        ramp = hMap.servo.get("ramp");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

        if (blockerR.getPosition() < 0.45 && blockerL.getPosition() > 0.55){
            blockersStatus = BlockersStatus.twoOpen;

        }else if (blockerR.getPosition() < 0.45 || blockerL.getPosition() > 0.55){
            blockersStatus = BlockersStatus.oneOpen;

        }else {
            blockersStatus = BlockersStatus.closed;
        }

        telemetry.addData("blockerStatus", blockersStatus);
    }

    public void setLateralPositions(double rightPos, double leftPos) {
        blockerR.setPosition(0.5 + rightPos);
        blockerL.setPosition(0.5 - leftPos);
    }

    public void setHorizontalPos(double Pos) {
        blockerH.setPosition(Pos);
    }

    public void setRampPos(double Pos) {
        ramp.setPosition(Pos);
    }
}
