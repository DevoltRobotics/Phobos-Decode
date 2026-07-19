package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Pepito extends OpMode {

    DcMotor motor1;



    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motorIntake");

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

    }
}
