package org.firstinspires.ftc.teamcode.tests;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Configurable
public class lifting_servo extends OpMode {

    Servo liftR, liftL;

    public static double servoPos = 0.3;

    //PIDFController principalTurretController = new PIDFController(principalTurretCoeffs);


    @Override
    public void init() {

        liftR = hardwareMap.get(Servo.class, "liftR");
        liftL = hardwareMap.get(Servo.class, "liftL");

    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            liftR.setPosition(0.5);
            liftL.setPosition(0.5);
        }else if (gamepad2.b) {
            liftR.setPosition(0.5 + servoPos);
            liftL.setPosition(0.5 - servoPos);
        }
    }
}
