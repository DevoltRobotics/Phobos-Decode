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

    public Telemetry telemetry;

    //CONSTANTS

    public static double blockersUp = -0.17;

    public static double blockerHHidePos = 0.41;
    public static double blockerHFreePos = 0.75;

    public static int waitAimTimer = 600;
    public static int artifacToArtifactTimer = 1300;

    enum BlockersStatus {
        closed,
        oneOpen,
        twoOpen

    }

    BlockersStatus blockersStatus = BlockersStatus.closed;

    public SorterSubsystem(HardwareMap hMap, Telemetry telemetry) {
        blockerR = hMap.servo.get("blcR");
        blockerL = hMap.servo.get("blcL");
        blockerH = hMap.servo.get("blcH");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

        if (blockerR.getPosition() > 0.55 && blockerL.getPosition() < 0.45){
            blockersStatus = BlockersStatus.twoOpen;

        }else if (blockerR.getPosition() > 0.55 || blockerL.getPosition() < 0.45){
            blockersStatus = BlockersStatus.oneOpen;

        }else {
            blockersStatus = BlockersStatus.closed;
        }

    }
}
