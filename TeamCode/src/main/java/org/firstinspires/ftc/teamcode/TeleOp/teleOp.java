package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.downRampPos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.manualIncrement;

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
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.aimCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;

public abstract class teleOp extends OpModeCommand {

    GamepadEx chasis;
    GamepadEx garra;

    double angleOffSet;
    Boolean isTurretManual = true;

    Boolean isShooting = false;

    public teleOp(Aliance alliance) {
        super(alliance, false);
    }

    @Override
    public void initialize() {
        follower.setPose(new Pose(72, 72, 0));

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

        Trigger turretRight = new Trigger(() -> gamepad2.right_bumper);
        Trigger turretLeft = new Trigger(() -> gamepad2.left_bumper);

        turretRight.whileActiveOnce(new InstantCommand(
                () -> shooterSb.setTurretTarget(shooterSb.getCurrentPosition() + manualIncrement)));
        turretLeft.whileActiveOnce(new InstantCommand(
                () -> shooterSb.setTurretTarget(shooterSb.getCurrentPosition() - manualIncrement)));

        Button resetTurretButton = new GamepadButton(
                garra,
                GamepadKeys.Button.START);

        resetTurretButton.whenPressed(new InstantCommand(() -> shooterSb.resetTurret()));

        CommandScheduler.getInstance().setDefaultCommand(sensorsSb, new lightSorterCMD(sensorsSb, shooterSb, visionSb));

        //CommandScheduler.getInstance().setDefaultCommand(sorterSb, new preSorterTeleopCMD(sorterSb, sensorsSb, gamepad2));

        Button blockerUpButton = new GamepadButton(
                garra,
                GamepadKeys.Button.Y);

        blockerUpButton.whileHeld(
                new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, blockersUp))
        );

        Button blockerDownButton = new GamepadButton(
                garra,
                GamepadKeys.Button.X);

        blockerDownButton.whileHeld(
                new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, 0))

        );

        Button prepareShootFar = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_LEFT);

        prepareShootFar.whenPressed(
                new ParallelCommandGroup(

                        new aimCMD(shooterSb),


                        new InstantCommand(
                                () -> isTurretManual = false
                        ),

                        new InstantCommand(
                                () -> isShooting = false
                        )

                ));

        Button prepareShootClose = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_RIGHT);

        prepareShootClose.whenPressed(
                new ParallelCommandGroup(

                        new aimCMD(shooterSb),

                        new InstantCommand(
                                () -> isShooting = false
                        ),

                        new InstantCommand(
                                () -> isTurretManual = false
                        )

                ));

        Button shootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_UP);

        shootButton.whenPressed(
                new ParallelCommandGroup(

                        new aimCMD(shooterSb),

                        new InstantCommand(
                                () ->
                                        isTurretManual = false
                        ),

                        new InstantCommand(
                                () -> isShooting = true
                        ),

                        new InstantCommand(
                                () -> sorterSb.isShooting = true
                        ),

                        new moveIntakeAutonomousCMD(intakeSb, 1, 1),

                        new horizontalBlockerCMD(sorterSb, blockerHFreePos).asProxy())
                );

        Button stopShootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_DOWN);

        stopShootButton.whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> shooterSb.setShooterTarget(800)
                        ),
                        new moveIntakeAutonomousCMD(intakeSb, 0, 0),

                        new InstantCommand(()-> shooterSb.setTurretTarget(0)),
                        new SequentialCommandGroup(
                                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                                new ConditionalCommand(
                                        new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, blockersUp)),
                                        new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, 0)),
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
                    if (Artifact.Green.equals(sensorsSb.targetArtifact)) {
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
                                new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, blockersUp)),
                                new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, 0)),
                                () -> sensorsSb.sorterMode

                        ),

                        new ConditionalCommand(
                                new InstantCommand(()-> sorterSb.setRampPos(downRampPos)),
                                new InstantCommand(()-> sorterSb.setRampPos(upRampPos)),
                                () -> sensorsSb.sorterMode

                        )
                )
        );

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
