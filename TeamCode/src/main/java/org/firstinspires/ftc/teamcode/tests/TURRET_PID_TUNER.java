package org.firstinspires.ftc.teamcode.tests;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;

@TeleOp
@Configurable
public class TURRET_PID_TUNER extends OpModeCommand {

    GamepadEx garra;

    public static double turretTargetTest = 40;
    public TURRET_PID_TUNER() {
        super(Alliance.RED, false);
    }

    @Override
    public void initialize() {

        garra = new GamepadEx(gamepad2);

       Button turretRight = new GamepadButton(
                garra,
                GamepadKeys.Button.A);

        turretRight.whenPressed(new InstantCommand(()-> shooterSb.setTurretTarget(turretTargetTest)));

        Button turretLeft = new GamepadButton(
                garra,
                GamepadKeys.Button.B);

        turretLeft.whenPressed(new InstantCommand(()-> shooterSb.setTurretTarget(-turretTargetTest)));


    }

    @Override
    public void start() {
        follower.setMaxPower(1);
        startCMD().schedule();
    }

    @Override
    public void run() {
        PedroSubsystem.EndPose = follower.getPose();

        telemetry.addData("Heading", Math.toDegrees(follower.poseTracker.getPose().getHeading()));

        telemetry.addData("EndPose", PedroSubsystem.EndPose);

    }
}
