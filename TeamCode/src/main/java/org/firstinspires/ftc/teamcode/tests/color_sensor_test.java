package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class color_sensor_test extends OpMode {

    int Scale = 1;

    RevColorSensorV3 colorSensor;

    @Override
    public void init() {

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

    }

    @Override
    public void loop() {

        telemetry.addData("red", colorSensor.red() * Scale);
        telemetry.addData("blue", colorSensor.blue() * Scale);
        telemetry.addData("green", colorSensor.green() * Scale);


    }
}
