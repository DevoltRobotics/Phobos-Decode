package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.preSorterCmd;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Config
@TeleOp
public class SorterComanndTest extends OpModeCommand {

    //GamepadEx chasis;
    GamepadEx garra;

    double angleOffSet;

    public SorterComanndTest() {
        super(Alliance.RED, false);
    }

    @Override
    public void initialize() {
        follower.setStartingPose(new Pose(pedroSb.EndPose.getX(), pedroSb.EndPose.getY(), pedroSb.EndPose.getHeading() + angleOffSet));

        //chasis = new GamepadEx(gamepad1);
        garra = new GamepadEx(gamepad2);

        ///CHASSIS

        Button blockerUpButton = new GamepadButton(
                garra,
                GamepadKeys.Button.Y);

        blockerUpButton.whenPressed(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp)
        );

        Button blockerDownButton = new GamepadButton(
                garra,
                GamepadKeys.Button.X);

        blockerDownButton.whenPressed(
                new lateralBlockersCMD(sorterSb, 0, blockersUp)
        );

        Button sorterSeq = new GamepadButton(
                garra,
                GamepadKeys.Button.A);

        /*sorterSeq.whileHeld(
                new preSorterCmd(sorterSb, visionSb)
        );


         */
    }

    @Override
    public void start() {
        startCMD().schedule();
    }

}
