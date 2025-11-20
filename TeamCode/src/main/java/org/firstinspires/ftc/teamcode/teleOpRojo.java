package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Alliance;

@TeleOp
public class teleOpRojo extends teleOp {
    public teleOpRojo() {
        super(Alliance.RED);
        angleOffSet = 0;
    }
}
