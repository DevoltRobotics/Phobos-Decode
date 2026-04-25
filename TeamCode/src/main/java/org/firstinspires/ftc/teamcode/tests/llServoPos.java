package org.firstinspires.ftc.teamcode.tests;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Configurable
public class llServoPos extends OpMode {

    Servo llS;

    public static double servoPos1 = 0.42;
    public static double servoPos2 = 0.47;


    @Override
    public void init() {

        llS = hardwareMap.get(Servo.class, "llS");

    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            llS.setPosition(servoPos1);
        }else if (gamepad2.b) {
            llS.setPosition(servoPos2);
        }
    }
}
