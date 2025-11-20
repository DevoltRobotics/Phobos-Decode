package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Alliance;

@TeleOp
public class teleOpAzul extends teleOp {
    public teleOpAzul() {
        super(Alliance.BLUE);

        angleOffSet = Math.PI;
    }
}
