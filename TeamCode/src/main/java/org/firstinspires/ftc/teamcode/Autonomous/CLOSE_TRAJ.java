package org.firstinspires.ftc.teamcode.Autonomous;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Autonomous
public class CLOSE_TRAJ extends OpModeCommand {

    public CLOSE_TRAJ() {
        super(Alliance.RED, true);
    }

    private Path launchPreload, park;
    private PathChain intakeFirst, launchFirst, intakeSecond, launchSecond, openGate1, intakeThird, launchThird, openGate2, intakeFourth, launchFourth, openGate3, intakeFive, launchFive;

    private Pose currentStartingPose;
    Command autoCommand;

    Pose m(Pose p) {
        return Alliance.BLUE.equals(currentAlliance) ? p.mirror() : p;
    }

    Pose startingPose = m(new Pose(110.0, 134.5, Math.toRadians(0)));

    Pose shootPreloadPose = m(new Pose(87.0, 85.0, Math.toRadians(320)));
    Pose pickUp1ControlPoint = m(new Pose(92.0, 56.0, Math.toRadians(0)));
    Pose pickUp1Pose = m(new Pose(134.0, 57.0, Math.toRadians(0)));

    Pose shoot1ControlPoint = m(new Pose(105.0, 62.0, Math.toRadians(0)));
    Pose shoot1Pose = m(new Pose(88.0, 81.0, Math.toRadians(320)));

    Pose openGate1ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate1Pose = m(new Pose(126.0, 64.0, Math.toRadians(0)));

    Pose pickUp2Pose = m(new Pose(131.0, 53.0, Math.toRadians(40)));
    Pose shoot2Pose = m(new Pose(88.0, 81.0, Math.toRadians(320)));

    Pose openGate2ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate2Pose = m(new Pose(126.0, 64.0, Math.toRadians(0)));

    Pose pickUp3Pose = m(new Pose(131.0, 53.0, Math.toRadians(40)));
    Pose shoot3Pose = m(new Pose(88.0, 81.0, Math.toRadians(320)));

    Pose openGate3ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate3Pose = m(new Pose(126.0, 64.0, Math.toRadians(0)));

    Pose pickUp4Pose = m(new Pose(131.0, 53.0, Math.toRadians(40)));
    Pose shoot4Pose = m(new Pose(88.0, 83.0, Math.toRadians(0)));
    Pose pickUp5Pose = m(new Pose(125.0, 84.0, Math.toRadians(0)));

    Pose shoot5Pose = m(new Pose(92.0, 85.0, Math.toRadians(0)));
    Pose parkPose = m(new Pose(115.0, 85.0, Math.toRadians(0)));

    PathChain fullAuto;

    public void createPaths() {


        launchPreload = new Path(new BezierLine(startingPose, shootPreloadPose));
        launchPreload.setConstantHeadingInterpolation(startingPose.getHeading());
        intakeFirst = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPreloadPose,
                        pickUp1ControlPoint,
                        pickUp1Pose))
                .setLinearHeadingInterpolation(shootPreloadPose.getHeading(), pickUp1Pose.getHeading())
                .build();

        launchFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickUp1Pose,
                        shoot1Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shoot1Pose,
                        openGate1ControlPoint,
                        openGate1Pose))
                .setTangentHeadingInterpolation()
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        openGate1Pose,
                        pickUp2Pose))
                .setLinearHeadingInterpolation(
                        openGate1Pose.getHeading(),
                        pickUp2Pose.getHeading()
                )
                .build();

        launchSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickUp2Pose,
                        shoot2Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shoot2Pose,
                        openGate2ControlPoint,
                        openGate2Pose))
                .setTangentHeadingInterpolation()
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(new BezierLine(
                        openGate2Pose,
                        pickUp3Pose))
                .setLinearHeadingInterpolation(
                        openGate2Pose.getHeading(),
                        pickUp3Pose.getHeading())
                .build();

        launchThird = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickUp3Pose,
                        shoot3Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shoot3Pose,
                        openGate3ControlPoint,
                        openGate3Pose))
                .setTangentHeadingInterpolation()
                .build();

        intakeFourth = follower.pathBuilder()
                .addPath(new BezierLine(
                        openGate3Pose,
                        pickUp4Pose))
                .setLinearHeadingInterpolation(
                        openGate3Pose.getHeading(),
                        pickUp4Pose.getHeading())
                .build();

        launchFourth = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickUp4Pose,
                        shoot4Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeFive = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot4Pose,
                        pickUp5Pose))
                .setConstantHeadingInterpolation(
                        pickUp5Pose.getHeading())
                .build();

        launchFive = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickUp5Pose,
                        shoot5Pose))
                .setConstantHeadingInterpolation(
                        pickUp5Pose.getHeading()
                )
                .build();

        park = new Path(new BezierLine(shoot5Pose, parkPose));
        park.setConstantHeadingInterpolation(shoot5Pose.getHeading());

    }


    @Override
    public void initialize() {

        if (currentAlliance.equals(Alliance.RED)) {
            follower.setStartingPose(startingPose);

        } else {
            follower.setStartingPose(startingPose.mirror());

        }

        createPaths();

        autoCommand =
                new SequentialCommandGroup(

                        pedroSb.followPathCmd(launchPreload),

                        pedroSb.followPathCmd(intakeFirst),

                        pedroSb.followPathCmd(launchFirst),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(openGate1),
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new InstantCommand(() -> follower.setMaxPower(0.6))
                                )),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        pedroSb.followPathCmd(intakeSecond),

                        new WaitCommand(500),

                        pedroSb.followPathCmd(launchSecond),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(openGate2),
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new InstantCommand(() -> follower.setMaxPower(0.6))
                                )),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        pedroSb.followPathCmd(intakeThird),

                        new WaitCommand(500),

                        pedroSb.followPathCmd(launchThird),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(openGate3),
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new InstantCommand(() -> follower.setMaxPower(0.6))
                                )),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        pedroSb.followPathCmd(intakeFourth),

                        new WaitCommand(500),

                        pedroSb.followPathCmd(launchFourth),

                        new InstantCommand(() -> follower.setMaxPower(0.8)),

                        pedroSb.followPathCmd(intakeFive),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        pedroSb.followPathCmd(launchFive),

                        pedroSb.followPathCmd(park)
                );

    }

    @Override
    public void start() {
        autoCommand.schedule();

    }

}

