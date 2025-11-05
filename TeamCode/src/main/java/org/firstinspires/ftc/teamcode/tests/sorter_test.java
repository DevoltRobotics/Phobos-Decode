package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class sorter_test extends OpMode {

    DcMotorEx intakeM;

    Servo blockerR;
    Servo blockerL;

//Adding each val with a key

    @Override
    public void init() {


        intakeM = hardwareMap.get(DcMotorEx.class, "in");

        blockerR = hardwareMap.get(Servo.class, "blcr");
        blockerL = hardwareMap.get(Servo.class, "blcl");
    }

    @Override
    public void loop() {

        intakeM.setPower(gamepad2.right_stick_y);

        if (gamepad2.a){
            blockerR.setPosition(0.5 + blockersUp);
            blockerL.setPosition(0.5 - blockersUp);
        }else if (gamepad2.b){
            blockerR.setPosition(0.5);
            blockerL.setPosition(0.5);
        }

        if (gamepad2.right_bumper){
            blockerR.setPosition(0.5 + blockersUp);
            blockerL.setPosition(0.5);

        }else if (gamepad2.left_bumper){
            blockerR.setPosition(0.5);
            blockerL.setPosition(0.5 - blockersUp);
        }

    }
}
