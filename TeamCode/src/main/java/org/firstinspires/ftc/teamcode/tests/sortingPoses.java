package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class sortingPoses extends OpMode {
    Servo blcR, blcL;
    @Override
    public void init() {

        blcL = hardwareMap.get(Servo.class, "blcL");
        blcR = hardwareMap.get(Servo.class, "blcR");

    }

    @Override
    public void loop() {

        if (gamepad2.a){
            blcR.setPosition(0.5);
            blcL.setPosition(0.5);

        }else if (gamepad2.b){
            blcR.setPosition(0.7);
            blcL.setPosition(0.3);
        }

    }
}
