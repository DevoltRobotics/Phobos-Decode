package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.controller.PIDFController;
@Disabled

@TeleOp
public class Torreta_onlypower extends OpMode {

    CRServo capstaneSr;

    AnalogInput axonE;

    //PIDFController turretController = new PIDFController(turretCoeffs);


    @Override
    public void init() {
        axonE = hardwareMap.get(AnalogInput.class, "axonE");

        capstaneSr = hardwareMap.get(CRServo.class, "trt");

    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            capstaneSr.setPower(0.85);

        } else if (gamepad2.left_bumper) {
            capstaneSr.setPower(-0.85);

        } else {
            capstaneSr.setPower(0);

        }
    }
}
