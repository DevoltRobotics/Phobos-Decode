package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretManaulCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;

@Config
@TeleOp
public class TurretTest extends OpModeCommand {

    GamepadEx garra;

    public static double turretTargetPos = 50;

    @Override
    public void initialize() {
        garra = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().setDefaultCommand(turretSb, new turretManaulCMD(turretSb, gamepad2));

        Button targetBasket = new GamepadButton(
                garra,
                GamepadKeys.Button.A);

        targetBasket.whenPressed(new turretToBasketCMD(turretSb, visionSb));
    }
}
