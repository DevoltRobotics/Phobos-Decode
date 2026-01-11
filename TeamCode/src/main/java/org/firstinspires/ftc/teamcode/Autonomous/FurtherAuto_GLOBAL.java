package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
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
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretHoldCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.detectMotifCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

public class FurtherAuto_GLOBAL extends OpModeCommand {

    private Path intakeFirst, park;
    private PathChain launchFirst, prepareForIntakeSecond, intakeSecond1, backIntakeSecond, intakeSecond2, launchSecond;


    static Pose startingPose = new Pose(88.0, 7.5, Math.toRadians(90));
    static Pose controlPick1Point = new Pose(86.5, 37.0, Math.toRadians(0));
    static Pose pick1Pose = new Pose(130.0, 35.0, Math.toRadians(0));
    static Pose shoot1Pose = new Pose(87.0, 15.0, Math.toRadians(25));
    static Pose preparePick2Pose = new Pose(115.0, 8.0, Math.toRadians(0));
    static Pose pick2Pose = new Pose(135.0, 8.0, Math.toRadians(0));
    static Pose shoot2Pose = new Pose(87.0, 15.0, Math.toRadians(350));

    Command autoCommand;

    public FurtherAuto_GLOBAL(Aliance aliance) {
        super(aliance, true);
    }

    public void createPaths() {

        if (currentAliance == Aliance.RED) {

            intakeFirst = new Path(new BezierCurve(startingPose, controlPick1Point, pick1Pose));
            intakeFirst.setTangentHeadingInterpolation();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick1Pose,
                            shoot1Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            prepareForIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot1Pose,
                            preparePick2Pose))
                    .setLinearHeadingInterpolation(
                            launchFirst.endPose().getHeading(),
                            preparePick2Pose.getHeading())
                    .build();

            intakeSecond1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            preparePick2Pose,
                            pick2Pose))
                    .setConstantHeadingInterpolation(
                            pick2Pose.getHeading())
                    .build();

            backIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick2Pose,
                            preparePick2Pose))
                    .setConstantHeadingInterpolation(
                            pick2Pose.getHeading())
                    .build();

            intakeSecond2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            preparePick2Pose,
                            pick2Pose))
                    .setConstantHeadingInterpolation(
                            pick2Pose.getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick2Pose,
                            shoot2Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            /*park = new Path(new BezierLine(launchSecondFurtherRedPose, parkFurtherRedPose));
            park.setLinearHeadingInterpolation(launchSecondFurtherRedPose.getHeading(), parkFurtherRedPose.getHeading());


             */

        } else {
            /*

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


             */
        }


    }

    @Override
    public void initialize() {
        if (currentAliance.equals(Aliance.RED)) {
            follower.setStartingPose(startingPose);

        } else {
            follower.setStartingPose(startingPose.mirror());

        }


        new SequentialCommandGroup(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                new WaitCommand(200),

                new lateralBlockersCMD(sorterSb, 0, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new WaitCommand(300),

                new detectMotifCMD(visionSb, 500)

        ).schedule();

        createPaths();


        autoCommand =
                new SequentialCommandGroup(

                        new shooterToBasketCMD(shooterSb, turretSb),

                        new ParallelRaceGroup(

                                new SequentialCommandGroup(

                                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                                        new preSorterCmd(sorterSb, sensorsSb, visionSb, 0.3),

                                        new moveIntakeAutonomousCMD(intakeSb, 1, 0.7),

                                        new WaitCommand(600),

                                        new moveIntakeAutonomousCMD(intakeSb, 0.5, -0.3),
                                        new WaitCommand(150),
                                        new postSorterCmd(sorterSb, sensorsSb, visionSb),
                                        new moveIntakeAutonomousCMD(intakeSb, 1)

                                ),

                                new SequentialCommandGroup(
                                        new turretToBasketCMD(turretSb),
                                        new WaitCommand(400)
                                )
                        ),

                        new ParallelRaceGroup(

                                new turretHoldCMD(turretSb),

                                new SequentialCommandGroup(
                                        new WaitCommand(500),

                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                                        new ParallelDeadlineGroup(
                                                new WaitCommand(1000), // deadline

                                                new SequentialCommandGroup(
                                                        new WaitCommand(500),
                                                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp)

                                                )
                                        )
                                )),

                        ///PRELOAD_LAUNCHED

                        stopShootCMD(),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(0.7)
                        ),

                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(intakeFirst, 2300),

                                new SequentialCommandGroup(
                                        new WaitCommand( 800),
                                new moveIntakeAutonomousCMD(intakeSb, 1, 0.7)
                                )                              ),

                        new WaitCommand(300),

                        sorter3CMD(launchFirst),

                        shootThreeCMD(),

                        ///FIRST_LAUNCHED

                        stopShootCMD(),

                        pedroSb.followPathCmd(prepareForIntakeSecond),

                        new moveIntakeAutonomousCMD(intakeSb, 1),

                       pedroSb.followPathCmd(intakeSecond1, 1000),

                        pedroSb.followPathCmd(backIntakeSecond, 500),

                        pedroSb.followPathCmd(intakeSecond2, 1000),

                        new WaitCommand(300),

                        sorter3CMD(launchSecond),

                        shootThreeCMD(),

                        /// SECOND_LAUNCHED

                        stopShootCMD(),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(1)
                        )

                        //pedroSb.followPathCmd(park)

                        ///PARK


                );


    }

    @Override
    public void start() {
        visionSb.getCurrentCommand().cancel();
        autoCommand.schedule();

    }
}
