package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHSortingPos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.downRampPos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.manualIncrement;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.EndPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.intakeWidth;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.xBlueCloseCorner;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.xBlueFarCorner;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.xRedCloseCorner;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.xRedFarCorner;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.yBlueCloseCorner;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.yBlueFarCorner;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.yRedCloseCorner;
import static org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem.yRedFarCorner;

import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.lightSorterCMD;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.aimCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;

@Configurable
public abstract class teleOp extends OpModeCommand {

    GamepadEx chassis;
    GamepadEx garra;

    double angleOffSet;
    boolean isShooting = false;

    boolean preparingShoot = false;

    boolean isClose = true;
    public static int timerSorting = 400;

    public teleOp(Alliance alliance) {
        super(alliance, false, true);
    }

    @Override
    public void initialize() {
        //INIT_CMDS

        //follower.setPose(new Pose(7.5, 8.2, 0));

        follower.setPose(EndPose);

        chassis = new GamepadEx(gamepad1);
        garra = new GamepadEx(gamepad2);


        ///CHASSIS

        CommandScheduler.getInstance().setDefaultCommand(pedroSb, pedroSb.fieldCentricCmd(gamepad1, angleOffSet));


        Button upChassis = new GamepadButton(
                chassis,
                GamepadKeys.Button.DPAD_UP);

        upChassis.whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> follower.setPose(new Pose(xRedFarCorner - intakeWidth, yRedFarCorner, angleOffSet))),
                        new InstantCommand(() -> follower.setPose(new Pose(xBlueFarCorner + intakeWidth, yBlueFarCorner, angleOffSet))),
                        () -> Alliance.RED.equals(currentAlliance))

        );

        Button rightChassis = new GamepadButton(
                chassis,
                GamepadKeys.Button.DPAD_RIGHT);

        rightChassis.whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> follower.setPose(new Pose(xBlueFarCorner, yBlueFarCorner, angleOffSet))),
                        new InstantCommand(() -> follower.setPose(new Pose(xRedFarCorner, yRedFarCorner, angleOffSet))),
                        () -> Alliance.RED.equals(currentAlliance))
        );

        Button downChassis = new GamepadButton(
                chassis,
                GamepadKeys.Button.DPAD_DOWN);

        downChassis.whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> follower.setPose(new Pose(xBlueCloseCorner, yBlueCloseCorner, angleOffSet))),
                        new InstantCommand(() -> follower.setPose(new Pose(xRedCloseCorner, yRedCloseCorner, angleOffSet))),
                        () -> Alliance.RED.equals(currentAlliance))
        );

        Button leftChassis = new GamepadButton(
                chassis,
                GamepadKeys.Button.DPAD_LEFT);

        leftChassis.whenPressed(
                new ConditionalCommand(
                        new InstantCommand(() -> follower.setPose(new Pose(xRedCloseCorner, yRedCloseCorner, angleOffSet))),
                        new InstantCommand(() -> follower.setPose(new Pose(xBlueCloseCorner, yBlueCloseCorner, angleOffSet))),
                        () -> Alliance.RED.equals(currentAlliance))
        );

        /// GARRA

        CommandScheduler.getInstance().setDefaultCommand(sensorsSb, new lightSorterCMD(sensorsSb, shooterSb, visionSb, () -> preparingShoot, gamepad2));

        /// //////////////////////////////////

        Trigger intakeIn = new Trigger(() -> gamepad2.right_trigger >= 0.3);
        Trigger intakeOut = new Trigger(() -> gamepad2.left_trigger >= 0.3);

        intakeIn.whileActiveOnce(new ConditionalCommand(
                new ConditionalCommand(
                        new moveIntakeTeleOpCMD(intakeSb, 1, 1),
                        new moveIntakeTeleOpCMD(intakeSb, 1, -0.8),
                        () -> isShooting),

                new ConditionalCommand(
                        new ConditionalCommand(
                                new moveIntakeTeleOpCMD(intakeSb, 1, 0.8),
                                new moveIntakeTeleOpCMD(intakeSb, 1, 1),
                                ()-> isRobotFar),
                        new moveIntakeTeleOpCMD(intakeSb, 1, 0.8),
                        () -> isShooting),
                () -> sensorsSb.sorterMode));

        intakeOut.whileActiveOnce(new moveIntakeTeleOpCMD(intakeSb, -0.8, -1));

        Button resetTurretButton = new GamepadButton(
                garra,
                GamepadKeys.Button.RIGHT_STICK_BUTTON);

        resetTurretButton.whenPressed(new InstantCommand(() -> shooterSb.resetTurret()));

        Button blockerUpButton = new GamepadButton(
                garra,
                GamepadKeys.Button.Y);

        blockerUpButton.whileHeld(
                new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, blockersUp))
        );

        Button leftBlockerDownButton = new GamepadButton(
                garra,
                GamepadKeys.Button.X);

        leftBlockerDownButton.whileHeld(
                new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, 0))

        );

        Button twoBlockersDown = new GamepadButton(
                garra,
                GamepadKeys.Button.B);

        twoBlockersDown.whileHeld(
                new InstantCommand(() -> sorterSb.setLateralPositions(0, 0))

        );

        CommandScheduler.getInstance().setDefaultCommand(shooterSb, new aimCMD(shooterSb, ()-> isShooting, ()-> isClose));

        Button prepareShootFar = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_LEFT);

        prepareShootFar.whenPressed(
                new ParallelCommandGroup(

                        new InstantCommand(
                                () -> preparingShoot = true
                        ),

                        new InstantCommand(
                                () -> isShooting = false
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
                                () -> isShooting = false
                        ),

                        new InstantCommand(
                                () -> preparingShoot = true
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
                        new InstantCommand(
                                () -> isShooting = true
                        ),

                        new InstantCommand(
                                () -> preparingShoot = true
                        ),

                        new ConditionalCommand(
                                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),
                                new moveIntakeAutonomousCMD(intakeSb, 1, 1),
                                ()-> isRobotFar),

                        new ConditionalCommand(
                                shootThreesorterCMD(timerSorting),
                                new InstantCommand(),
                                () -> sensorsSb.sorterMode
                        ),
                        new horizontalBlockerCMD(sorterSb, blockerHFreePos).asProxy())
        );

        Button stopShootButton = new GamepadButton(
                garra,
                GamepadKeys.Button.DPAD_DOWN);

        stopShootButton.whenPressed(
                new ParallelCommandGroup(

                        new moveIntakeAutonomousCMD(intakeSb, 0, 0),

                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHSortingPos)),
                                        new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHHidePos)),
                                        () -> sensorsSb.sorterMode

                                ),

                                new ConditionalCommand(
                                        new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, blockersUp)),
                                        new InstantCommand(() -> sorterSb.setLateralPositions(blockersUp, 0)),
                                        () -> sensorsSb.sorterMode

                                )
                        ),

                        new InstantCommand(
                                () -> isShooting = false
                        )
                        ));

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
                                new InstantCommand(() -> sorterSb.setRampPos(downRampPos)),
                                new InstantCommand(() -> sorterSb.setRampPos(upRampPos)),
                                () -> sensorsSb.sorterMode

                        ),

                        new ConditionalCommand(
                                new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHSortingPos)),
                                new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHHidePos)),
                                () -> sensorsSb.sorterMode

                        )
                )
        );

        Button toggleSorterTarget = new GamepadButton(
                garra,
                GamepadKeys.Button.A);

        toggleSorterTarget.whenPressed(switchPatternTarget());

    }

    @Override
    public void start() {
        follower.setMaxPower(1);
        startCMD().schedule();
    }

    @Override
    public void run() {
        PedroSubsystem.EndPose = follower.getPose();

        telemetry.addLine("-----------------------------");
        telemetry.addData("EndPose", PedroSubsystem.EndPose);
    }
}
