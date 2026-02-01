package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;

@Config
public class SensorsSubsystem extends SubsystemBase {

    public Servo light;

    DigitalChannel laserInput;

    public boolean rightDetected = false;
    public boolean leftDetected = false;
    public boolean fourDetected = false;

    boolean laserState = false;


    public Boolean sorterMode = false;
    public Boolean liftingMode = false;

    RevColorSensorV3 colorR;
    RevColorSensorV3 colorL;

    public double greenInR = 0;
    public double greenInL = 0;

    public double blueInR = 0;
    public double blueInL = 0;

    public static int targetBlueRight = 130;
    public static int targetGreenRight = 160;

    public static int targetBlueLeft = 145;
    public static int targetGreenLeft = 190;

    public Artifact currentRightArtifact = null;
    public Artifact currentLeftArtifact = null;

    public Artifact lastRightArtifact = null;
    public Artifact lastLeftArtifact = null;


    public enum RelaseOrder{
        RL,
        LR,
        RR,
        LL
    }

    public RelaseOrder relaseOrder;

    public Artifact targetArtifact = Artifact.Purple;

    private final ElapsedTime colorSensorsDelay;

    Telemetry telemetry;

    public SensorsSubsystem(HardwareMap hMap, Telemetry telemetry){
        light = hMap.get(Servo.class, "rgb");
        light.setPosition(0);

        laserInput = hMap.get(DigitalChannel.class, "laser");
        laserInput.setMode(DigitalChannel.Mode.INPUT);


        colorR = hMap.get(RevColorSensorV3.class, "colorR");
        colorL = hMap.get(RevColorSensorV3.class, "colorL");

        colorSensorsDelay = new ElapsedTime();

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        if (colorSensorsDelay.milliseconds() > 80) {
            greenInR = colorR.green();
            greenInL = colorL.green();
            blueInR = colorR.blue();
            blueInL = colorL.blue();

            laserState = laserInput.getState();

            colorSensorsDelay.reset();
        }

        if (greenInR > targetGreenRight && greenInR > blueInR) {
            currentRightArtifact = Artifact.Green;
            rightDetected = true;

        } else if (blueInR > targetBlueRight && blueInR > greenInR) {
            currentRightArtifact = Artifact.Purple;
            rightDetected = true;

        }else {
            currentRightArtifact = null;
            rightDetected = false;
        }

        if (greenInL > targetGreenLeft && greenInL > blueInL) {
            currentLeftArtifact = Artifact.Green;
            leftDetected = true;

        } else if (blueInL > targetBlueLeft && blueInL > greenInL) {
            currentLeftArtifact = Artifact.Purple;
            leftDetected = true;

        }else {
            currentLeftArtifact = null;
            leftDetected = false;
        }

        telemetry.addData("RightGreen", greenInR);
        telemetry.addData("LeftGreen", greenInL);
        telemetry.addData("RightBlue", blueInR);
        telemetry.addData("LeftBlue", blueInL);

        telemetry.addData("WhatIsRightArt", currentRightArtifact);
        telemetry.addData("WhatIsLefttArt", currentLeftArtifact);


        fourDetected = (rightDetected && leftDetected) && laserState;


        if (fourDetected) {
            telemetry.addLine("FourBallDetected");

        } else {
            telemetry.addLine("NoFourBall");

        }

        telemetry.addData("relaseOrder", relaseOrder);


    }

}
