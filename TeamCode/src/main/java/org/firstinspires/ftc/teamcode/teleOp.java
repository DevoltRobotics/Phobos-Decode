package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.artifacToArtifactTimer;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.waitAimTimer;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.lightSorterCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.preSorterTeleopCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Config
public abstract class teleOp extends OpModeCommand {

    //GamepadEx chasis;
    GamepadEx garra;

    double angleOffSet;
    Boolean isTurretManual = true;
    boolean isClose = true;

    public teleOp(Alliance alliance) {
        super(alliance, false);
    }

    @Override
    public void initialize() {
        follower.setStartingPose(new Pose(pedroSb.EndPose.getX(), pedroSb.EndPose.getY(), pedroSb.EndPose.getHeading() + angleOffSet));

        //chasis = new GamepadEx(gamepad1);
        garra = new GamepadEx(gamepad2);

        ///CHASSIS

        CommandScheduler.getInstance().setDefaultCommand(pedroSb, pedroSb.fieldCentricCmd(gamepad1));

        /// GARRA

        Trigger intakeIn = new Trigger(() -> gamepad2.right_trigger >= 0.5);
        Trigger intakeOut = new Trigger(() -> gamepad2.left_trigger >= 0.5);

        intakeIn.whileActiveOnce(new moveIntakeCMD(intakeSb, 1));
        intakeOut.whileActiveOnce(new moveIntakeCMD(intakeSb, -0.7));

        CommandScheduler.getInstance().setDefaultCommand(turretSb, new turretToBasketCMD(turretSb, visionSb, sensorsSb, follower, () -> isTurretManual, () -> isClose, gamepad2));

        Button blockerUpButton = new GamepadButton(
                garra,
                GamepadKeys.Button.Y);

        blockerUpButton.whileHeld(new lateralBlockersCMD(sorterSb, blockersUp, blockersUp));


        Button blockerDownButton = new GamepadButton(
                garra,
                GamepadKeys.Button.X);

        blockerDownButton.whenPressed(

                new ConditionalCommand(
                        new lateralBlockersCMD(sorterSb, 0, 0),
                        new lateralBlockersCMD(sorterSb, 0, blockersUp),
                        () -> sensorsSb.sorterMode

                )


        );

        Button prepareShootCloseButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_RIGHT);

        prepareShootCloseButton.whenPressed(
                new ParallelCommandGroup(
                        new shooterToVelCMD(shooterSb, 1200),

                        new InstantCommand(
                                () -> isTurretManual = false
                        ),

                        new InstantCommand(
                                () -> isClose = true
                        )
                ));

        Button prepareShootFurtherButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_LEFT);

        prepareShootFurtherButton.whenPressed(
                new ParallelCommandGroup(
                        new shooterToVelCMD(shooterSb, 1400),

                        new InstantCommand(
                                () -> isTurretManual = false
                        ),

                        new InstantCommand(
                                () -> isClose = false
                        )
                ));

        Button shootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_UP);

        shootButton.whenPressed(
                new ParallelCommandGroup(
                        new shooterToBasketCMD(shooterSb, visionSb),
                        new InstantCommand(
                                () -> {
                                    isTurretManual = false;
                                }
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(waitAimTimer + 100),
                                new moveIntakeAutonomousCMD(intakeSb, 1)
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(waitAimTimer),
                                new horizontalBlockerCMD(sorterSb, blockerHFreePos).asProxy(),
                                new WaitCommand(artifacToArtifactTimer)
                        )
                ));

        Button stopShootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_DOWN);

        stopShootButton.whenPressed(
                new ParallelCommandGroup(
                        new shooterToVelCMD(shooterSb, 0),
                        new moveIntakeAutonomousCMD(intakeSb, 0),

                        new SequentialCommandGroup(
                                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                                new ConditionalCommand(
                                        new lateralBlockersCMD(sorterSb, 0, 0),
                                        new lateralBlockersCMD(sorterSb, 0, blockersUp),
                                        () -> sensorsSb.sorterMode

                               )
            ),

                        new InstantCommand(
                                () -> isTurretManual = true
                        )
                ));

        CommandScheduler.getInstance().setDefaultCommand(sensorsSb, new lightSorterCMD(sensorsSb, visionSb));

        CommandScheduler.getInstance().setDefaultCommand(sorterSb, new preSorterTeleopCMD(sorterSb, sensorsSb));

        Button toggleSorterTarget = new GamepadButton(
                garra,
                GamepadKeys.Button.A);

        toggleSorterTarget.whenPressed(
                        new InstantCommand(() -> {
                            if (sensorsSb.targetArtifact.equals(Artifact.Green)) {
                                sensorsSb.targetArtifact = Artifact.Purple;

                            } else {
                                sensorsSb.targetArtifact = Artifact.Green;

                            }
                        })
        );

        Button toggleSorterMode = new GamepadButton(
                garra,
                GamepadKeys.Button.PS);

        toggleSorterMode.whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> sensorsSb.sorterMode = !sensorsSb.sorterMode
                        ),

                        new WaitCommand(80),

                        new ConditionalCommand(
                                new lateralBlockersCMD(sorterSb, 0, 0),
                                new lateralBlockersCMD(sorterSb, 0, blockersUp),
                                () -> sensorsSb.sorterMode

                        )
                )
        );

    }

    @Override
    public void start() {

        startCMD().schedule();
    }

}
