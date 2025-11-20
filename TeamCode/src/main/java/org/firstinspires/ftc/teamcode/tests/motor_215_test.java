package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class motor_215_test extends OpMode {

    DcMotorEx intake;

    @Override
    public void init() {

        intake = hardwareMap.get(DcMotorEx.class, "in");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad2.a) {
            intake.setTargetPosition(751);
        } else if (gamepad2.b) {
            intake.setPower(0);
        }

    }
}
