package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class lifting_servo extends OpMode {

    CRServo liftS;


    //PIDFController principalTurretController = new PIDFController(principalTurretCoeffs);


    @Override
    public void init() {

        liftS = hardwareMap.get(CRServo.class, "liftS");

    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            liftS.setPower(1);

        } else if (gamepad2.left_bumper) {
            liftS.setPower(-1);

        } else {
            liftS.setPower(0);

        }
    }
}
