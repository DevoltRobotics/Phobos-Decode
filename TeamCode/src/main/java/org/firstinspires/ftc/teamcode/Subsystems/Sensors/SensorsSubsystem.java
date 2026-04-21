package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

import java.util.function.BooleanSupplier;

@Configurable
public class SensorsSubsystem extends SubsystemBase {

    public Servo lightR;
    public Servo lightL;

    DigitalChannel laserInput;

    public boolean rightDetected = false;
    public boolean leftDetected = false;
    public boolean fourDetected = false;

    boolean laserState = false;

    public Boolean sorterMode = false;
    public Boolean liftingMode = false;

    RevColorSensorV3 colorR;
    RevColorSensorV3 colorL;

    public float greenInR = 0;
    public float greenInL = 0;

    public float blueInR = 0;
    public float blueInL = 0;

    public static int targetBlueRight = 115;
    public static int targetGreenRight = 205;

    public static int targetBlueLeft = 190;
    public static int targetGreenLeft = 150;

    public Artifact currentRightArtifact = null;
    public Artifact currentLeftArtifact = null;

    public Artifact lastRightArtifact = null;
    public Artifact lastLeftArtifact = null;

    //LIGHT

    public static double lightGreen = 0.5;
    public static double lightPurple = 0.7;
    public static double lightRed = 0.28;
    public static double lightBlue = 0.65;

    public enum RelaseOrder{
        RL,
        LR,
        RR,
        LL
    }

    public RelaseOrder relaseOrder;

    public Artifact targetArtifact = Artifact.Purple;

    public Pattern teleOpPattern = Pattern.PPG;
    private final ElapsedTime colorSensorsDelay;

    Telemetry telemetry;

    public SensorsSubsystem(HardwareMap hMap, Telemetry telemetry){
        lightR = hMap.get(Servo.class, "rgbR");
        lightR.setPosition(0);

        lightL = hMap.get(Servo.class, "rgbL");
        lightL.setPosition(0);

        laserInput = hMap.get(DigitalChannel.class, "laser");
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        colorR = hMap.get(RevColorSensorV3.class, "colorR");
        colorL = hMap.get(RevColorSensorV3.class, "colorL");

        colorSensorsDelay = new ElapsedTime();
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        /*if (colorSensorsDelay.milliseconds() > 80) {
            if (sorterMode) {
                greenInR = colorR.green();
                greenInL = colorL.green();
                blueInR = colorR.blue();
                blueInL = colorL.blue();
            }

            colorSensorsDelay.reset();
        }

         */

        laserState = laserInput.getState();

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

        telemetry.addData("relaseOrder", relaseOrder);

        telemetry.addData("pattern", teleOpPattern);
    }

    public void setLightPos(double pos){
        lightR.setPosition(pos);
        lightL.setPosition(pos);
    }

    public void setLightPos(double posR, double posL){
        lightR.setPosition(posR);
        lightL.setPosition(posL);
    }

}
