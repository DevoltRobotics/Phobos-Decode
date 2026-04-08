package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Configurable
public class PTO_test extends OpMode {

    DcMotorEx fr, br, bl, fl;

    Servo servoR, servoL;

    boolean isPTO = false;

    public static double servosMove = 0.25;

    @Override
    public void init() {
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        fl = hardwareMap.get(DcMotorEx.class, "fl");

        servoR = hardwareMap.get(Servo.class, "ptoR");
        servoL = hardwareMap.get(Servo.class, "ptoL");
    }

    @Override
    public void loop() {
        if (gamepad2.a){
            servoR.setPosition(0.5);
            servoL.setPosition(0.5);
            isPTO = false;
        }else if (gamepad2.b){
            servoR.setPosition(0.5 + servosMove);
            servoL.setPosition(0.5 - servosMove);

            isPTO = true;
        }

        double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad2.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad2.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if (isPTO){
            fl.setPower(gamepad2.left_stick_y);
            fr.setPower(-gamepad2.left_stick_y);

        }else {
            fl.setPower(frontLeftPower);
            bl.setPower(backLeftPower);
            fr.setPower(frontRightPower);
            br.setPower(backRightPower);

        }

        telemetry.addData("isPto", isPTO);
    }
}
