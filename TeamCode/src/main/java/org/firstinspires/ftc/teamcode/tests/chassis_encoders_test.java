package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class chassis_encoders_test extends OpMode {

    DcMotorEx FR;
    DcMotorEx BR;
    DcMotorEx BL;

    DcMotorEx FL;

    @Override
    public void init() {
        FR = hardwareMap.get(DcMotorEx.class, "fr");
        BR = hardwareMap.get(DcMotorEx.class, "br");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        FL = hardwareMap.get(DcMotorEx.class, "fl");

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

    }

    @Override
    public void loop() {
        telemetry.addData("fr", FR.getCurrentPosition());
        telemetry.addData("br", BR.getCurrentPosition());
        telemetry.addData("bL", BL.getCurrentPosition());
        telemetry.addData("fl", FL.getCurrentPosition());
        telemetry.update();
    }
}
