package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Aliance;

@TeleOp
public class teleOpAzul extends teleOp {
    public teleOpAzul() {
        super(Aliance.BLUE);

        angleOffSet = Math.PI;
    }
}
