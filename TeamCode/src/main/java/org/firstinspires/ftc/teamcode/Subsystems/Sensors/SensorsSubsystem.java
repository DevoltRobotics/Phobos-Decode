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

    public Boolean sorterMode = false;

    RevColorSensorV3 colorR;
    RevColorSensorV3 colorL;

    public double greenInR = 0;
    public double greenInL = 0;

    public double blueInR = 0;
    public double blueInL = 0;

    public Artifact rightArtifact = Artifact.Purple;
    public Artifact leftArtifact = Artifact.Green;

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

            fourDetected = laserInput.getState();

            colorSensorsDelay.reset();
        }

        if (greenInR > 100 && greenInR > blueInR) {
            rightArtifact = Artifact.Green;
            rightDetected = true;

        } else if (blueInR > 90 && blueInR > greenInR) {
            rightArtifact = Artifact.Purple;
            rightDetected = true;

        }else {
            rightDetected = false;
        }

        if (greenInL > 190 && greenInL > blueInL) {
            leftArtifact = Artifact.Green;
            leftDetected = true;

        } else if (blueInL > 170 && blueInL > greenInL) {
            leftArtifact = Artifact.Purple;
            leftDetected = true;

        }else {
            leftDetected = false;
        }

        telemetry.addData("RightGreen", greenInR);
        telemetry.addData("LeftGreen", greenInL);
        telemetry.addData("RightBlue", blueInR);
        telemetry.addData("LeftBlue", blueInL);

        telemetry.addData("WhatIsRightArt", rightArtifact);
        telemetry.addData("WhatIsLefttArt", leftArtifact);


        if (fourDetected) {
            telemetry.addLine("FourBallDetected");

        } else {
            telemetry.addLine("NoFourBall");

        }



        // Display the raw HIGH/LOW signal for reference
        telemetry.addData("Raw (HIGH/LOW)", fourDetected);


        telemetry.addData("targetArtifact", targetArtifact);
    }

}
