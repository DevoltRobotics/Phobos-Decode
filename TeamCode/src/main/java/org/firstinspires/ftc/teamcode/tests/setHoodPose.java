package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class setHoodPose extends OpMode {

    Servo hood;

    @Override
    public void init() {
        hood = hardwareMap.servo.get("hoodS");

    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            hood.setPosition(1);


        } else if (gamepad2.b) {
            hood.setPosition(0);
        }
    }

}
