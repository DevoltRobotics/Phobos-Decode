package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Test extends OpMode {

    DcMotor test, test2;

    @Override
    public void init() {
        test = hardwareMap.dcMotor.get("shup");
        test2 = hardwareMap.dcMotor.get("shdown");
        test.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        test.setPower(gamepad1.left_stick_y);
        test2.setPower(test.getPower());
    }
}
