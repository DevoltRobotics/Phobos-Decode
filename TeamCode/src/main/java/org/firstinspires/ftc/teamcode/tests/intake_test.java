package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled

@TeleOp
public class intake_test extends OpMode {

    DcMotorEx intake;

    @Override
    public void init() {

        intake = hardwareMap.get(DcMotorEx.class, "in");
    }

    @Override
    public void loop() {

        intake.setPower(gamepad2.right_stick_y);

    }
}
