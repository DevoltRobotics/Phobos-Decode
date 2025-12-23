package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeFirstCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeFirstFurtherBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeFirstFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeSecondCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeSecondCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeSecondFurtherBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeSecondFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeThirdCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeThirdCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstFurtherBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchPreloadCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchPreloadCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondControlPointCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondControlPointCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondFurtherBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchThirdCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchThirdCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.openGateCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.openGateCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.openGateFirstControlPointCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.openGateFirstControlPointCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.openGateSecondControlPointCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.openGateSecondControlPointCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkFurtherBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstControlPointFurtherBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstControlPointFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstFurtherBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeSecondCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeSecondCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeSecondFurtherBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeSecondFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeThirdCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeThirdCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseBlue;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseRed;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseFurtherBlue;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseFurtherRed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.postSorterCmd;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.preSorterCmd;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketFurtherAutoCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.detectMotifCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

public class FurtherAuto_GLOBAL extends OpModeCommand {

    private Path prepareForIntakeFirst, park;
    private PathChain intakeFirst, launchFirst, prepareForIntakeSecond, intakeSecond, launchSecond;

    Command autoCommand;

    public FurtherAuto_GLOBAL(Aliance aliance) {
        super(aliance, true);
    }

    public void createPaths() {

        if (currentAliance == Aliance.RED) {

            prepareForIntakeFirst = new Path(new BezierCurve(startingPoseFurtherRed, prepareForIntakeFirstControlPointFurtherRedPose, prepareForIntakeFirstFurtherRedPose));
            prepareForIntakeFirst.setLinearHeadingInterpolation(
                    startingPoseFurtherRed.getHeading(),
                    prepareForIntakeFirstFurtherRedPose.getHeading());

            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            prepareForIntakeFirstFurtherRedPose,
                            intakeFirstFurtherRedPose))
                    .setLinearHeadingInterpolation(prepareForIntakeFirstFurtherRedPose.getHeading(), intakeFirstFurtherRedPose.getHeading())
                    .build();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            intakeFirstFurtherRedPose,
                            launchFirstFurtherRedPose))
                    .setLinearHeadingInterpolation(intakeFirstFurtherRedPose.getHeading(), launchFirstFurtherRedPose.getHeading())
                    .build();

            prepareForIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            launchFirstFurtherRedPose,
                            prepareForIntakeSecondFurtherRedPose))
                    .setLinearHeadingInterpolation(
                            launchFirstFurtherRedPose.getHeading(),
                            prepareForIntakeSecondFurtherRedPose.getHeading())
                    .build();

            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            prepareForIntakeSecondFurtherRedPose,
                            intakeSecondFurtherRedPose))
                    .setLinearHeadingInterpolation(
                            prepareForIntakeSecondFurtherRedPose.getHeading(),
                            intakeSecondFurtherRedPose.getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            intakeSecondFurtherRedPose,
                            launchSecondFurtherRedPose))
                    .setLinearHeadingInterpolation(
                            intakeSecondFurtherRedPose.getHeading(),
                            launchSecondFurtherRedPose.getHeading())
                    .build();

            park = new Path(new BezierLine(launchSecondFurtherRedPose, parkFurtherRedPose));
            park.setLinearHeadingInterpolation(launchSecondFurtherRedPose.getHeading(), parkFurtherRedPose.getHeading());


        } else {

            prepareForIntakeFirst = new Path(new BezierCurve(startingPoseFurtherBlue, prepareForIntakeFirstControlPointFurtherBluePose, prepareForIntakeFirstFurtherBluePose));
            prepareForIntakeFirst.setLinearHeadingInterpolation(
                    startingPoseFurtherBlue.getHeading(),
                    prepareForIntakeFirstFurtherBluePose.getHeading());

            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            prepareForIntakeFirstFurtherBluePose,
                            intakeFirstFurtherBluePose))
                    .setLinearHeadingInterpolation(
                            prepareForIntakeFirstFurtherBluePose.getHeading(),
                            intakeFirstFurtherBluePose.getHeading())
                    .build();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            intakeFirstFurtherBluePose,
                            launchFirstFurtherBluePose))
                    .setLinearHeadingInterpolation(
                            intakeFirstFurtherBluePose.getHeading(),
                            launchFirstFurtherBluePose.getHeading())
                    .build();

            prepareForIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            launchFirstFurtherBluePose,
                            prepareForIntakeSecondFurtherBluePose))
                    .setLinearHeadingInterpolation(
                            launchFirstFurtherBluePose.getHeading(),
                            prepareForIntakeSecondFurtherBluePose.getHeading())
                    .build();

            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            prepareForIntakeSecondFurtherBluePose,
                            intakeSecondFurtherBluePose))
                    .setLinearHeadingInterpolation(
                            prepareForIntakeSecondFurtherBluePose.getHeading(),
                            intakeSecondFurtherBluePose.getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            intakeSecondFurtherBluePose,
                            launchSecondFurtherBluePose))
                    .setLinearHeadingInterpolation(
                            intakeSecondFurtherBluePose.getHeading(),
                            launchSecondFurtherBluePose.getHeading())
                    .build();

            park = new Path(new BezierLine(launchSecondFurtherBluePose, parkFurtherBluePose));
            park.setLinearHeadingInterpolation(
                    launchSecondFurtherBluePose.getHeading(),
                    parkFurtherBluePose.getHeading());

        }
    }

    @Override
    public void initialize() {
        if (currentAliance.equals(Aliance.RED)) {
            follower.setStartingPose(startingPoseFurtherRed);

        } else {
            follower.setStartingPose(startingPoseFurtherBlue);

        }

        new SequentialCommandGroup(
                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -8),
                        new turretToPosCMD(turretSb, 8),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                new WaitCommand(400),
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new WaitCommand(200),
                new lateralBlockersCMD(sorterSb, 0, 0),

                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new WaitCommand(300),

                new detectMotifCMD(visionSb, 500)

        ).schedule();

        createPaths();

        autoCommand =
                new SequentialCommandGroup(

                        new shooterToVelCMD(shooterSb, 1430),

                        new ParallelRaceGroup(

                                new SequentialCommandGroup(

                                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                                        new preSorterCmd(sorterSb, sensorsSb, visionSb, 0.3),

                                        new moveIntakeAutonomousCMD(intakeSb, 1),

                                        new WaitCommand(1200),

                                        new moveIntakeAutonomousCMD(intakeSb, -0.6),
                                        new WaitCommand(150),
                                        new postSorterCmd(sorterSb, sensorsSb, visionSb),
                                        new moveIntakeAutonomousCMD(intakeSb, 1)

                                ),

                                new SequentialCommandGroup(
                                        new ConditionalCommand(
                                                new turretToPosCMD(turretSb, 22),
                                                new turretToPosCMD(turretSb, -22),
                                                () -> currentAliance.equals(Aliance.RED)
                                        ),
                                        new WaitCommand(400)
                                )
                        ),

                        new WaitCommand(1450),

                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                        new ParallelDeadlineGroup(
                                new WaitCommand(4000), // deadline

                                new SequentialCommandGroup(
                                        new WaitCommand(2200),
                                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp)

                                )
                        ),

                        ///PRELOAD_LAUNCHED

                        new moveIntakeAutonomousCMD(intakeSb, 0),
                        new shooterToVelCMD(shooterSb, 0),
                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                        new lateralBlockersCMD(sorterSb, 0, 0),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.8)
                        ),

                        pedroSb.followPathCmd(prepareForIntakeFirst),

                        new moveIntakeAutonomousCMD(intakeSb, 1),

                        new lateralBlockersCMD(sorterSb, 0, 0),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.7)
                        ),

                        new ParallelRaceGroup(
                                pedroSb.followPathCmd(intakeFirst),
                                new WaitCommand(2300)
                        ),

                        new WaitCommand(300),

                        new moveIntakeAutonomousCMD(intakeSb, 0),
                        new WaitCommand(100),

                        new preSorterCmd(sorterSb, sensorsSb, visionSb, 0.3),

                        new moveIntakeAutonomousCMD(intakeSb, 1),

                        new shooterToVelCMD(shooterSb, 1380),

                        new ConditionalCommand(
                                new turretToPosCMD(turretSb, -72),
                                new turretToPosCMD(turretSb, 72),
                                () -> currentAliance.equals(Aliance.RED)
                        ),

                        new WaitCommand(400),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.9)
                        ),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(launchFirst),

                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        new moveIntakeAutonomousCMD(intakeSb, -0.6),
                                        new WaitCommand(150),
                                        new postSorterCmd(sorterSb, sensorsSb, visionSb),
                                        new moveIntakeAutonomousCMD(intakeSb, 1)

                                )
                        ),

                        new ParallelDeadlineGroup(

                                new WaitCommand(4000), // deadline

                                new SequentialCommandGroup(
                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                                        new WaitCommand(2000),
                                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp)

                                ),

                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new moveIntakeAutonomousCMD(intakeSb, 1)
                                )
                        ),

                        ///FIRST_LAUNCHED

                        new moveIntakeAutonomousCMD(intakeSb, 0),
                        new shooterToVelCMD(shooterSb, 0),
                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                        new lateralBlockersCMD(sorterSb, 0, 0),

                        pedroSb.followPathCmd(prepareForIntakeSecond),

                        new moveIntakeAutonomousCMD(intakeSb, 1),

                        new ParallelRaceGroup(
                                pedroSb.followPathCmd(intakeSecond),
                                new WaitCommand(2200)
                        ),

                        new WaitCommand(300),

                        new moveIntakeAutonomousCMD(intakeSb, 0),
                        new WaitCommand(100),

                        new preSorterCmd(sorterSb, sensorsSb, visionSb, 0.3),

                        new moveIntakeAutonomousCMD(intakeSb, 1),
                        new shooterToVelCMD(shooterSb, 1380),

                        new ConditionalCommand(
                                new turretToPosCMD(turretSb, -70),
                                new turretToPosCMD(turretSb, 70),
                                () -> currentAliance.equals(Aliance.RED)
                        ),

                        new WaitCommand(400),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(launchSecond),

                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        new moveIntakeAutonomousCMD(intakeSb, -0.6),
                                        new WaitCommand(150),
                                        new postSorterCmd(sorterSb, sensorsSb, visionSb),
                                        new moveIntakeAutonomousCMD(intakeSb, 1)

                                )
                        ),

                        new ParallelDeadlineGroup(

                                new WaitCommand(4000), // deadline

                                new SequentialCommandGroup(
                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                                        new WaitCommand(2000),
                                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp)

                                ),

                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new moveIntakeAutonomousCMD(intakeSb, 1)
                                )
                        ),

                        /// SECOND_LAUNCHED

                        new shooterToVelCMD(shooterSb, 0),
                        new turretToPosCMD(turretSb, 0),
                        new moveIntakeAutonomousCMD(intakeSb, 0),

                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                        new lateralBlockersCMD(sorterSb, 0, 0),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(1)
                        ),

                        pedroSb.followPathCmd(park)

                        ///PARK


                );





                        /*

        autoCommand =
                new SequentialCommandGroup(

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.8)
                        ),

                        pedroSb.followPathCmd(prepareForIntakeFirst),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.7)
                        ),

                        pedroSb.followPathCmd(intakeFirst),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.8)
                        ),

                        pedroSb.followPathCmd(launchFirst),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.8)
                        ),

                        pedroSb.followPathCmd(prepareForIntakeSecond),

                        pedroSb.followPathCmd(intakeSecond),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.8)
                        ),

                        pedroSb.followPathCmd(launchSecond)

                );


                         */


    }

    @Override
    public void start() {
        visionSb.getCurrentCommand().cancel();
        autoCommand.schedule();

    }
}
