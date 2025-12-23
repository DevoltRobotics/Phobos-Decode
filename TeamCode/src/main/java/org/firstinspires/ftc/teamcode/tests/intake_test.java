package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config

@TeleOp
public class intake_test extends OpMode {

    DcMotor intakeM;
    DcMotor transferM;

    CRServo intakeS;

    Servo ramp;

    public static double upPos = 0.8;
    public static double downPos = 0.5;


    @Override
    public void init() {

        intakeM = hardwareMap.dcMotor.get("in");
        intakeM.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeS = hardwareMap.get(CRServo.class,"star");
        intakeS.setDirection(DcMotorSimple.Direction.REVERSE);


        transferM = hardwareMap.dcMotor.get("trans");
        transferM.setDirection(DcMotorSimple.Direction.REVERSE);

        ramp = hardwareMap.servo.get("ramp");
    }

    @Override
    public void loop() {

        if (gamepad2.right_trigger > 0.5){
            intakeM.setPower(1);
            intakeS.setPower(1);
            transferM.setPower(1);

        }else if (gamepad2.left_trigger > 0.5){
            intakeM.setPower(-1);
            intakeS.setPower(-1);
            transferM.setPower(-1);

        }else {
            intakeM.setPower(0);
            intakeS.setPower(0);
            transferM.setPower(0);

        }

        if (gamepad2.a){
            ramp.setPosition(upPos);

        }else if (gamepad2.b){
            ramp.setPosition(downPos);

        }


    }
}
