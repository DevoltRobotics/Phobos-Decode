package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class lifting_test extends OpMode {

    DcMotorEx intake;

    @Override
    public void init() {

        intake = hardwareMap.get(DcMotorEx.class, "lift");
    }

    @Override
    public void loop() {

        if (gamepad2.a){
            intake.setPower(1);

        }else if (gamepad2.b){
            intake.setPower(-1);

        }else {
            intake.setPower(0);
        }


    }
}
