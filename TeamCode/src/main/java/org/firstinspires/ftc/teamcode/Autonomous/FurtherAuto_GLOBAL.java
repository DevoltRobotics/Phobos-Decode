package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeFirstCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeFirstFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeSecondCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeSecondCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeSecondFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeThirdCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeThirdCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchPreloadCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchPreloadCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondControlPointCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondControlPointCloseRedPose;
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
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstControlPointFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeSecondCloseBluePose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeSecondCloseRedPose;
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

    private Path park;
    private PathChain prepareForIntakeFirst, intakeFirst, launchFirst, prepareForIntakeSecond, intakeSecond, launchSecond, prepareForIntakeThird, intakeThird, launchThird;

    Command autoCommand;

    public FurtherAuto_GLOBAL(Aliance aliance) {
        super(aliance, true);
    }

    public void createPaths() {

        if (currentAliance == Aliance.RED) {

            prepareForIntakeFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            startingPoseFurtherRed,
                            prepareForIntakeFirstControlPointFurtherRedPose,
                            prepareForIntakeFirstFurtherRedPose))
                    .setLinearHeadingInterpolation(
                            startingPoseFurtherRed.getHeading(),
                            prepareForIntakeFirstFurtherRedPose.getHeading())
                    .build();

            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            prepareForIntakeFirstFurtherRedPose,
                            intakeFirstFurtherRedPose))
                    .setConstantHeadingInterpolation(intakeFirstFurtherRedPose.getHeading())
                    .build();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            intakeFirstFurtherRedPose,
                            launchFirstFurtherRedPose))
                    .setConstantHeadingInterpolation(launchFirstFurtherRedPose.getHeading())
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

        } else {


            prepareForIntakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(launchPreloadCloseBluePose, prepareForIntakeFirstCloseBluePose))
                    .setConstantHeadingInterpolation(prepareForIntakeFirstCloseBluePose.getHeading())
                    .build();

            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(prepareForIntakeFirstCloseBluePose, intakeFirstCloseBluePose))
                    .setLinearHeadingInterpolation(prepareForIntakeFirstCloseBluePose.getHeading(), intakeFirstCloseBluePose.getHeading())

                    .build();


            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(openGateCloseBluePose, launchFirstCloseBluePose))
                    .setLinearHeadingInterpolation(openGateCloseBluePose.getHeading(), launchFirstCloseBluePose.getHeading())

                    .build();

            prepareForIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(launchFirstCloseBluePose, prepareForIntakeSecondCloseBluePose))
                    .setLinearHeadingInterpolation(launchFirstCloseBluePose.getHeading(), prepareForIntakeSecondCloseBluePose.getHeading())
                    .build();

            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(prepareForIntakeSecondCloseBluePose, intakeSecondCloseBluePose))
                    .setConstantHeadingInterpolation(intakeSecondCloseBluePose.getHeading())

                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierCurve(intakeSecondCloseBluePose, launchSecondControlPointCloseBluePose, launchSecondCloseBluePose))
                    .setLinearHeadingInterpolation(intakeSecondCloseBluePose.getHeading(), launchSecondCloseBluePose.getHeading())

                    .build();

            prepareForIntakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(launchSecondCloseBluePose, prepareForIntakeThirdCloseBluePose))
                    .setLinearHeadingInterpolation(launchSecondCloseBluePose.getHeading(), prepareForIntakeThirdCloseBluePose.getHeading())
                    .build();

            intakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(prepareForIntakeThirdCloseBluePose, intakeThirdCloseBluePose))
                    .setLinearHeadingInterpolation(prepareForIntakeThirdCloseBluePose.getHeading(), intakeThirdCloseBluePose.getHeading())

                    .build();

            launchThird = follower.pathBuilder()
                    .addPath(new BezierLine(intakeThirdCloseBluePose, launchThirdCloseBluePose))
                    .setLinearHeadingInterpolation(intakeThirdCloseBluePose.getHeading(), launchThirdCloseBluePose.getHeading())
                    .build();

            park = new Path(new BezierLine(launchThirdCloseBluePose, parkCloseBluePose));
            park.setLinearHeadingInterpolation(launchThirdCloseBluePose.getHeading(), parkCloseBluePose.getHeading());

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
                new turretToPosCMD(turretSb, -8),
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new WaitCommand(200),
                new lateralBlockersCMD(sorterSb, 0, 0),

                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new WaitCommand(300),

                new detectMotifCMD(visionSb, 4000)

        ).schedule();

        createPaths();

        autoCommand =
                new SequentialCommandGroup(

                        new shooterToVelCMD(shooterSb, 1440),

                        new ParallelRaceGroup(

                                new SequentialCommandGroup(

                                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                                        new preSorterCmd(sorterSb, sensorsSb, visionSb, 0.4),

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

                        new WaitCommand(1400),

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

                        pedroSb.followPathCmd(intakeFirst),
                        new WaitCommand(300),

                        new preSorterCmd(sorterSb, sensorsSb, visionSb, 0.4),

                        new shooterToVelCMD(shooterSb, 1400),

                        new ConditionalCommand(
                                new turretToPosCMD(turretSb, -72),
                                new turretToPosCMD(turretSb, 72),
                                () -> currentAliance.equals(Aliance.RED)
                        ),

                        new WaitCommand(400),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.8)
                        ),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(launchFirst),

                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        new postSorterCmd(sorterSb, sensorsSb, visionSb)

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
                                new WaitCommand(1500)
                        ),

                        new WaitCommand(300),

                        new preSorterCmd(sorterSb, sensorsSb, visionSb, 0.4),

                        new shooterToVelCMD(shooterSb, 1400),

                        new ConditionalCommand(
                                new turretToPosCMD(turretSb, -72),
                                new turretToPosCMD(turretSb, 72),
                                () -> currentAliance.equals(Aliance.RED)
                        ),

                        new WaitCommand(400),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(launchSecond),

                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        new postSorterCmd(sorterSb, sensorsSb, visionSb)

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
                        )

                        /*

                        /// SECOND_LAUNCHED

                        new moveIntakeAutonomousCMD(intakeSb, 0),
                        new shooterToVelCMD(shooterSb, 0),
                        pedroSb.followPathCmd(prepareForIntakeThird),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.5)
                        ),

                        new ParallelCommandGroup(

                                pedroSb.followPathCmd(intakeThird),
                                new moveIntakeAutonomousCMD(intakeSb, 1),
                                new preSorterCmd(sorterSb, sensorsSb, visionSb)

                        ),

                        new WaitCommand(1500),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.85)
                        ),

                        new ParallelCommandGroup(

                                pedroSb.followPathCmd(launchThird),

                                new SequentialCommandGroup(
                                        new postSorterCmd(sorterSb, sensorsSb, visionSb),
                                        new WaitCommand(400),
                                        new moveIntakeAutonomousCMD(intakeSb, 0)

                                ),

                                new shooterToVelCMD(shooterSb, 1230),
                                new turretToPosCMD(turretSb, -70)
                        ),

                        new ParallelDeadlineGroup(

                                new WaitCommand(3000), // deadline

                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                                        new WaitCommand(1800),
                                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp)

                                ),

                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new moveIntakeAutonomousCMD(intakeSb, 1)
                                ),

                                new turretToBasketCMD(turretSb, visionSb),
                                new shooterToBasketCMD(shooterSb, visionSb)

                        ),

                        ///THIRD_LAUNCHED

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(park),
                                new shooterToVelCMD(shooterSb, 0),
                                new turretToPosCMD(turretSb, 0),
                                new moveIntakeAutonomousCMD(intakeSb, 0),


                            new SequentialCommandGroup(
                                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                                        new lateralBlockersCMD(sorterSb, 0, 0)

                                )
                        )





                        ///PARK


                   v
                         */
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
        autoCommand.schedule();

    }
}
