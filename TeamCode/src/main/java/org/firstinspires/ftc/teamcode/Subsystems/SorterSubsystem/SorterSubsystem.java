package org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;

@Config
public class SorterSubsystem extends SubsystemBase {

    Servo blockerR;
    Servo blockerL;

    Servo blockerH;

    RevColorSensorV3 colorR;
    RevColorSensorV3 colorL;

    public double greenInR = 0;
    public double greenInL = 0;

    public double blueInR = 0;
    public double blueInL = 0;

    public Artifact rightArtifact = Artifact.Purple;
    public Artifact leftArtifact = Artifact.Green;

    private ElapsedTime colorSensorsDelay;

    public Telemetry telemetry;

    //CONSTANTS

    public static double blockersUp = -0.2;

    public static double blockerHHidePos = 0.45;
    public static double blockerHFreePos = 0.75;

    public static int waitAimTimer = 600;
    public static int artifacToArtifactTimer = 1300;

    public SorterSubsystem(HardwareMap hMap, Telemetry telemetry) {
        blockerR = hMap.servo.get("blcR");
        blockerL = hMap.servo.get("blcL");
        blockerH = hMap.servo.get("blcH");

        colorR = hMap.get(RevColorSensorV3.class, "colorR");
        colorL = hMap.get(RevColorSensorV3.class, "colorL");

        colorSensorsDelay = new ElapsedTime();

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

        /*
        if (colorSensorsDelay.milliseconds() > 100) {
            greenInR = colorR.green();
            greenInL = colorL.green();
            blueInR = colorR.blue();
            blueInL = colorL.blue();

            colorSensorsDelay.reset();
        }

        if (greenInR > 100 && greenInR > blueInR) {
            rightArtifact = Artifact.Green;
        } else if (blueInR > 90 && blueInR > greenInR) {
            rightArtifact = Artifact.Purple;
        }

        if (greenInL > 190 && greenInL > blueInL) {
            leftArtifact = Artifact.Green;
        } else if (blueInL > 170 && blueInL > greenInL) {
            leftArtifact = Artifact.Purple;
        }


        telemetry.addData("RightGreen", greenInR);
        telemetry.addData("LeftGreen", greenInL);
        telemetry.addData("RightBlue", blueInR);
        telemetry.addData("LeftBlue", blueInL);

        telemetry.addData("WhatIsRightArt", rightArtifact);
        telemetry.addData("WhatIsLefttArt", leftArtifact);

*/
    }
}
