package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Config
@Autonomous
public class FURTHER_TRAJ extends OpModeCommand {

    private Path intakeFirst, park;
    private PathChain launchFirst, prepareForIntakeSecond, intakeSecond1, backIntakeSecond, intakeSecond2, launchSecond;


    static Pose startingPose = new Pose(88.0, 7.5, Math.toRadians(90));
    static Pose controlPick1Point = new Pose(86.5, 37.0, Math.toRadians(0));
    static Pose pick1Pose = new Pose(130.0, 35.0, Math.toRadians(0));
    static Pose shoot1Pose = new Pose(87.0, 15.0, Math.toRadians(25));
    static Pose preparePick2Pose = new Pose(122.0, 8.0, Math.toRadians(0));
    static Pose pick2Pose = new Pose(135.0, 8.0, Math.toRadians(0));
    static Pose shoot2Pose = new Pose(90.0, 15.0, Math.toRadians(0));
    static Pose parkPose = new Pose(100.0, 25.0, Math.toRadians(0));

    Command autoCommand;

    public FURTHER_TRAJ() {
        super(Aliance.RED, true);
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
                    .setReversed()
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
                    .setConstantHeadingInterpolation(shoot2Pose.getHeading())
                    .build();

            park = new Path(new BezierLine(shoot2Pose, parkPose));
            park.setConstantHeadingInterpolation(parkPose.getHeading());


        } else {
            intakeFirst = new Path(new BezierCurve(
                    startingPose.mirror(),
                    controlPick1Point.mirror(),
                    pick1Pose.mirror()));
            intakeFirst.setTangentHeadingInterpolation();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick1Pose.mirror(),
                            shoot1Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            prepareForIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot1Pose.mirror(),
                            preparePick2Pose.mirror()))
                    .setLinearHeadingInterpolation(
                            launchFirst.endPose().getHeading(),
                            preparePick2Pose.mirror().getHeading())
                    .build();

            intakeSecond1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            preparePick2Pose.mirror(),
                            pick2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pick2Pose.mirror().getHeading())
                    .build();

            backIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick2Pose.mirror(),
                            preparePick2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pick2Pose.mirror().getHeading())
                    .build();

            intakeSecond2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            preparePick2Pose.mirror(),
                            pick2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pick2Pose.mirror().getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick2Pose.mirror(),
                            shoot2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            shoot2Pose.mirror().getHeading())
                    .build();

            park = new Path(new BezierLine(
                    shoot2Pose.mirror(),
                    parkPose.mirror()));
            park.setConstantHeadingInterpolation(
                    parkPose.mirror().getHeading());
        }


    }

    @Override
    public void initialize() {
        if (currentAliance.equals(Aliance.RED)) {
            follower.setStartingPose(startingPose);

        } else {
            follower.setStartingPose(startingPose.mirror());

        }

        /*new SequentialCommandGroup(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                new rampCMD(sorterSb, downRampPos),

                new WaitCommand(200),

                new lateralBlockersCMD(sorterSb, 0, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new WaitCommand(300),

                new detectMotifCMD(visionSb, 500)

        ).schedule();

         */



        createPaths();

        autoCommand =
                new SequentialCommandGroup(

                        pedroSb.followPathCmd(intakeFirst),

                        pedroSb.followPathCmd(launchFirst),
                        pedroSb.followPathCmd(prepareForIntakeSecond),

                        pedroSb.followPathCmd(intakeSecond1),
                        pedroSb.followPathCmd(backIntakeSecond),
                        pedroSb.followPathCmd(intakeSecond2),
                        pedroSb.followPathCmd(launchSecond),
                        pedroSb.followPathCmd(park)






                        );

        /*

        autoCommand =
                new SequentialCommandGroup(

                        new shooterToBasketCMD(shooterSb, turretSb, visionSb),

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

                        stopShootCMD(true),

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

                        shootThreeSorterCMD(),

                        ///FIRST_LAUNCHED

                        stopShootCMD(true),

                        pedroSb.followPathCmd(prepareForIntakeSecond),

                        new moveIntakeAutonomousCMD(intakeSb, 1),

                       pedroSb.followPathCmd(intakeSecond1, 1000),

                        pedroSb.followPathCmd(backIntakeSecond, 500),

                        pedroSb.followPathCmd(intakeSecond2, 1000),

                        new WaitCommand(300),

                        sorter3CMD(launchSecond),

                        shootThreeSorterCMD(),

                        /// SECOND_LAUNCHED

                        stopShootCMD(true),

                        new InstantCommand(
                                () -> pedroSb.follower.setMaxPower(1)
                        )

                        //pedroSb.followPathCmd(park)

                        ///PARK


                );


         */

    }

    @Override
    public void start() {
        //visionSb.getCurrentCommand().cancel();
        autoCommand.schedule();

    }
}
