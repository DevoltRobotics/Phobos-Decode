package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.artifacToArtifactTimer;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.downRampPos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;
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
import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeTeleOpCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Lifting.moveLiftCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.lightSorterCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketTeleOpCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.preSorterTeleopCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretDefaultTeleopCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Config
public abstract class teleOp extends OpModeCommand {

    static Pose startingPose = new Pose(88.0, 7.5, Math.toRadians(90));

    GamepadEx chasis;
    GamepadEx garra;

    double angleOffSet;
    Boolean isTurretManual = true;

    Boolean isShooting = true;

    Boolean isClose = true;

    Boolean poseEstimate = true;


    Double shooterProvTarget = 1300.0;

    public teleOp(Aliance alliance) {
        super(alliance, false);
    }

    @Override
    public void initialize() {
        //follower.setStartingPose(new Pose(pedroSb.EndPose.getX(), pedroSb.EndPose.getY(), pedroSb.EndPose.getHeading()));

       follower.setStartingPose(startingPose);

        chasis = new GamepadEx(gamepad1);
        garra = new GamepadEx(gamepad2);

        ///CHASSIS

        CommandScheduler.getInstance().setDefaultCommand(pedroSb, pedroSb.fieldCentricCmd(gamepad1, angleOffSet));

        Button toggleLiftingMode = new GamepadButton(
                chasis,
                GamepadKeys.Button.PS);

        toggleLiftingMode.whenPressed(
                new InstantCommand(
                        () -> sensorsSb.liftingMode = !sensorsSb.liftingMode
                ));

        Button liftingUp = new GamepadButton(
                chasis,
                GamepadKeys.Button.DPAD_UP);

        Button liftingDown = new GamepadButton(
                chasis,
                GamepadKeys.Button.DPAD_DOWN);

        liftingUp.whileActiveOnce(
                new moveLiftCMD(liftingSb, -1)
        );

        liftingDown.whileActiveOnce(
                new moveLiftCMD(liftingSb, 1)

        );

        /// GARRA

        Trigger intakeIn = new Trigger(() -> gamepad2.right_trigger >= 0.5);
        Trigger intakeOut = new Trigger(() -> gamepad2.left_trigger >= 0.5);

        intakeIn.whileActiveOnce(new moveIntakeTeleOpCMD(intakeSb, 1, 0.8));
        intakeOut.whileActiveOnce(new moveIntakeTeleOpCMD(intakeSb, -0.7, -1));

        CommandScheduler.getInstance().setDefaultCommand(turretSb, new turretDefaultTeleopCMD(turretSb, visionSb, () -> isTurretManual, () -> isClose, gamepad2, () -> isShooting, () -> poseEstimate));

        Button togglePoseEstimate = new GamepadButton(
                garra,
                GamepadKeys.Button.START);

        togglePoseEstimate.whenPressed(
                new InstantCommand(
                        () ->

                        {
                            if (poseEstimate != null) {
                                poseEstimate = !poseEstimate;

                            }
                        }
                )
        );


        CommandScheduler.getInstance().setDefaultCommand(sensorsSb, new lightSorterCMD(sensorsSb, shooterSb, visionSb));

        CommandScheduler.getInstance().setDefaultCommand(sorterSb, new preSorterTeleopCMD(sorterSb, sensorsSb, gamepad2));

        Button blockerUpButton = new GamepadButton(
                garra,
                GamepadKeys.Button.Y);

        blockerUpButton.whileHeld(new lateralBlockersCMD(sorterSb, blockersUp, blockersUp));

        Button blockerDownButton = new GamepadButton(
                garra,
                GamepadKeys.Button.X);

        blockerDownButton.whileHeld(
                new ConditionalCommand(
                        new lateralBlockersCMD(sorterSb, 0, 0),
                        new lateralBlockersCMD(sorterSb, blockersUp, 0),
                        () -> sensorsSb.sorterMode
                )

        );

        Button prepareShootFar = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_RIGHT);

        prepareShootFar.whenPressed(
                new ParallelCommandGroup(

                        new InstantCommand(
                                () -> shooterProvTarget = 1480.0

                        ),

                        new shooterToBasketTeleOpCMD(shooterSb, visionSb, turretSb, shooterProvTarget),


                        new InstantCommand(
                                () -> isTurretManual = false
                        ),

                        new InstantCommand(
                                () -> isShooting = true
                        ),

                        new InstantCommand(
                                () -> isClose = false
                        )


                ));

        Button prepareShootClose = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_RIGHT);

        prepareShootClose.whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> shooterProvTarget = 1480.0

                        ),

                        new shooterToBasketTeleOpCMD(shooterSb, visionSb, turretSb, shooterProvTarget),



                        new InstantCommand(
                                () -> isShooting = true
                        ),

                        new InstantCommand(
                                () -> isTurretManual = false
                        ),

                        new InstantCommand(
                                () -> isClose = true
                        )


                ));

        Button shootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_UP);

        shootButton.whenPressed(
                new ParallelCommandGroup(

                        new shooterToBasketTeleOpCMD(shooterSb, visionSb, turretSb, shooterProvTarget),


                        new InstantCommand(
                                () -> {
                                    isTurretManual = false;
                                }
                        ),

                        new InstantCommand(
                                () -> isShooting = true
                        ),

                        new InstantCommand(
                                () -> sorterSb.isShooting = true
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new moveIntakeAutonomousCMD(intakeSb, 1, 1),
                                new horizontalBlockerCMD(sorterSb, blockerHFreePos).asProxy()
                                )
                ));

        Button stopShootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_DOWN);

        stopShootButton.whenPressed(
                new ParallelCommandGroup(
                        new shooterToVelCMD(shooterSb, 500),
                        new moveIntakeAutonomousCMD(intakeSb, 0, 0),

                        new SequentialCommandGroup(
                                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                                new ConditionalCommand(
                                        new lateralBlockersCMD(sorterSb, 0, 0),
                                        new lateralBlockersCMD(sorterSb, blockersUp, 0),
                                        () -> sensorsSb.sorterMode

                                )
                        ),

                        new InstantCommand(
                                () -> isTurretManual = true
                        ),

                        new InstantCommand(
                                () -> isShooting = false
                        ),

                        new InstantCommand(
                                () -> sorterSb.isShooting = false
                        )
                ));

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
                                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                                () -> sensorsSb.sorterMode

                        ),

                        new ConditionalCommand(
                                new rampCMD(sorterSb, downRampPos),
                                new rampCMD(sorterSb, upRampPos),
                                () -> sensorsSb.sorterMode

                        )
                )
        );

/*
        Trigger openLeft = new Trigger(() -> gamepad2.right_stick_x <= 0.5);
        Trigger openRight = new Trigger(() -> gamepad2.right_stick_x >= 0.5);

        openLeft.whileActiveOnce(new lateralBlockersCMD(sorterSb, 0, blockersUp));
        openRight.whileActiveOnce(new lateralBlockersCMD(sorterSb, blockersUp, 0));


 */

    }

    @Override
    public void start() {

        follower.setMaxPower(1);
        startCMD().schedule();
    }
}
