package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class color_sensor_test extends OpMode {

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
}
