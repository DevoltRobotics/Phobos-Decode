package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Config
@TeleOp
public class TurretTest extends OpModeCommand {

    GamepadEx garra;

    public static double turretTargetPos = 50;

    public TurretTest() {
        super(Aliance.RED, false);
    }

    @Override
    public void initialize() {
        garra = new GamepadEx(gamepad2);

        //CommandScheduler.getInstance().setDefaultCommand(turretSb, new turretManaulCMD(turretSb, visionSb, follower, gamepad2));

        Button targetBasket = new GamepadButton(
                garra,
                GamepadKeys.Button.A);

        //targetBasket.whenPressed(new turretToBasketCMD(turretSb, visionSb, follower));
    }
}
