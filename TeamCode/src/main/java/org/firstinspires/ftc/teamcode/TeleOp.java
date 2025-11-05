package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem.standarShooterVel;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretManaulCMD;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Config
public class TeleOp extends OpModeCommand {

    GamepadEx chasis;

    GamepadEx garra;

    double intakePower = 0;
    @Override
    public void initialize() {
        chasis = new GamepadEx(gamepad1);
        garra = new GamepadEx(gamepad2);

        ///CHASSIS

        Button resetCentric = new GamepadButton(
                chasis,GamepadKeys.Button.DPAD_UP
        );

        resetCentric.whenPressed(new InstantCommand(()->
                //follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0)),
                follower.poseTracker.resetOffset()
        ));
        /// GARRA
        CommandScheduler.getInstance().setDefaultCommand(intakeSb, new moveIntakeCMD(intakeSb, intakePower));

        if (gamepad2.right_trigger > 0.5){
            intakePower = 1;
        }else if (gamepad2.left_trigger > 0.5){
            intakePower = -1;
        }else {
            intakePower = 0;
        }

        CommandScheduler.getInstance().setDefaultCommand(turretSb, new turretManaulCMD(turretSb, gamepad2));

        CommandScheduler.getInstance().setDefaultCommand(shooterSb, new shooterToVelCMD(shooterSb, standarShooterVel));


        Button targetBasket = new GamepadButton(
                garra,
                GamepadKeys.Button.A);


        //);
    }
}
