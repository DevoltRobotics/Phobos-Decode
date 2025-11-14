package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretManaulCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

import java.util.function.DoubleSupplier;

@TeleOp
@Config
public class teleOp extends OpModeCommand {

    //GamepadEx chasis;
    GamepadEx garra;

    double shooterSuplier = 0;

    public enum shooterStatusEnum {
    CLOSE,FURTHER,OFF
    };

    public shooterStatusEnum standarShooterVel = shooterStatusEnum.OFF;


    boolean sorterMode = false;
    ElapsedTime sorterModeTimer = new ElapsedTime();

    @Override
    public void initialize() {
        currentAliance = Alliance.RED;

        //chasis = new GamepadEx(gamepad1);
        garra = new GamepadEx(gamepad2);

        ///CHASSIS

        CommandScheduler.getInstance().setDefaultCommand(pedroSb, pedroSb.fieldCentricCmd(gamepad1));

        /// GARRA

        Trigger intakeIn = new Trigger(() -> gamepad2.right_trigger >= 0.5);
        Trigger intakeOut = new Trigger(() -> gamepad2.left_trigger >= 0.5);

        intakeIn.whileActiveOnce(new moveIntakeCMD(intakeSb, 1));
        intakeOut.whileActiveOnce(new moveIntakeCMD(intakeSb, -1));

        telemetry.addData("StandarVelStatus", standarShooterVel);


        CommandScheduler.getInstance().setDefaultCommand(turretSb, new turretManaulCMD(turretSb, gamepad2));

        CommandScheduler.getInstance().setDefaultCommand(shooterSb, new shooterToVelCMD(shooterSb, ()-> gamepad2.right_stick_y));

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

        Button autoAimButton = new GamepadButton(
                garra,
                GamepadKeys.Button.A);

        autoAimButton.whileHeld(
                new turretToBasketCMD(turretSb, visionSb)
        );

        Button prepareShootCloseButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_RIGHT);

        prepareShootCloseButton.whenPressed(new shooterToVelCMD(shooterSb, 1200));

        Button prepareShootFurtherButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_LEFT);

        prepareShootFurtherButton.whenPressed(new shooterToVelCMD(shooterSb, 1400));

        Button shootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_UP);

        shootButton.whenPressed(cyclesShootCMD(()-> (gamepad2.right_bumper || gamepad2.left_bumper)));

        Button stopShootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_DOWN);

        stopShootButton.whenPressed(
                new ParallelCommandGroup(
                        new shooterToVelAutonomousCMD(shooterSb, 0),
                        new moveIntakeAutonomousCMD(intakeSb, 0),

                new SequentialCommandGroup(
                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                        new lateralBlockersCMD(sorterSb, 0, blockersUp)
                )
        ));

    }

    @Override
    public void start() {
        startCMD().schedule();
    }

}
