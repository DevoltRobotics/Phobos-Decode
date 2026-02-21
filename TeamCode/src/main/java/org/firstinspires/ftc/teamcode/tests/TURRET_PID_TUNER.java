package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem.shooterCoeffs;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.trtTarget;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;

@TeleOp
public class TURRET_PID_TUNER extends OpModeCommand {

    GamepadEx garra;

    public TURRET_PID_TUNER() {
        super(Aliance.RED, false);
    }

    @Override
    public void initialize() {

        garra = new GamepadEx(gamepad2);

        Button turretRight = new GamepadButton(
                garra,
                GamepadKeys.Button.A);

        turretRight.whenPressed(new turretToPosCMD(turretSb, trtTarget));

        Button turretLeft = new GamepadButton(
                garra,
                GamepadKeys.Button.B);

        turretLeft.whenPressed(new turretToPosCMD(turretSb, -trtTarget));
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
