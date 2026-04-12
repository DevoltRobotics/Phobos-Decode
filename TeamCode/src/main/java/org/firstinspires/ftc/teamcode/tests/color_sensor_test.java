package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class color_sensor_test extends OpMode {


    NormalizedColorSensor colorR;
    NormalizedColorSensor colorL;

    public int SCALE_FACTOR = 10;


    @Override
    public void init() {

        colorR = hardwareMap.get(NormalizedColorSensor.class, "colorR");
        colorL = hardwareMap.get(NormalizedColorSensor.class, "colorL");

        colorR.setGain(4);
        colorL.setGain(4);
    }

    @Override
    public void loop() {
        getDetectedColors(telemetry);
    }

    public void getDetectedColors (Telemetry telemetry){

        NormalizedRGBA colorsR = colorR.getNormalizedColors();
        float normRRed, normRBlue, normRGreen;

        normRRed = (colorsR.red / colorsR.alpha) * SCALE_FACTOR;
        normRBlue = (colorsR.blue / colorsR.alpha) * SCALE_FACTOR;
        normRGreen = (colorsR.green / colorsR.alpha) * SCALE_FACTOR;

        NormalizedRGBA colorsL = colorL.getNormalizedColors();
        float normLRed, normLBlue, normLGreen;

        normLRed = (colorsL.red / colorsL.alpha) * SCALE_FACTOR;
        normLBlue = (colorsL.blue / colorsL.alpha) * SCALE_FACTOR;
        normLGreen = (colorsL.green / colorsL.alpha) * SCALE_FACTOR;

        telemetry.addData("Rightred", normRRed);
        telemetry.addData("Rightblue", normRBlue);
        telemetry.addData("Rightgreen", normRGreen);

        telemetry.addLine("--------------------");

        telemetry.addData("Leftred", normLRed);
        telemetry.addData("Leftblue", normLBlue);
        telemetry.addData("Leftgreen", normLGreen);
    }

    /*
    RevColorSensorV3 colorR;
    RevColorSensorV3 colorL;


    @Override
    public void init() {

        colorR = hardwareMap.get(RevColorSensorV3.class, "colorR");
        colorL = hardwareMap.get(RevColorSensorV3.class, "colorL");
    }

    @Override
    public void loop() {
        telemetry.addData("Rightred", colorR.red());
        telemetry.addData("Rightblue", colorR.blue());
        telemetry.addData("Rightgreen", colorR.green());

        telemetry.addLine("--------------------");

        telemetry.addData("Leftred", colorL.red());
        telemetry.addData("Leftblue", colorL.blue());
        telemetry.addData("Leftgreen", colorL.green());

    }
     */
}
