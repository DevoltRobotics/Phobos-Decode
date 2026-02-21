package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Config
@TeleOp
public class TurretTest extends OpMode {

    public static double turretTargetPos = 50;

    DcMotorEx trtM;

    @Override
    public void init() {
        trtM = hardwareMap.get(DcMotorEx.class, "trtM");

    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper){
            trtM.setPower(1);

        }else if (gamepad2.left_bumper){
            trtM.setPower(-1);

        }else {
            trtM.setPower(0);

        }

    }


}
