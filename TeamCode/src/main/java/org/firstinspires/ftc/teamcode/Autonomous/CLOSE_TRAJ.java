package org.firstinspires.ftc.teamcode.Autonomous;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Autonomous
public class CLOSE_TRAJ extends OpModeCommand {

    public CLOSE_TRAJ() {
        super(Alliance.RED, true, true);
    }

    private PathChain openGate1, openGate2, openGate3;
    private Path launchPreload, intakeFirst, launchFirst, launchSecond, launchThird, launchFourth, intakeFive, launchFive, park;

    private Pose currentStartingPose;
    Command autoCommand;

    Pose m(Pose p) {
        return Alliance.BLUE.equals(currentAlliance) ? p.mirror() : p;
    }

    Pose startingPose = m(new Pose(110.0, 133, Math.toRadians(0)));

    Pose shootPreloadPose = m(new Pose(87.0, 85.0, Math.toRadians(320)));

    Pose pickUp1ControlPoint = m(new Pose(88.0, 54.0, Math.toRadians(0)));
    Pose pickUp1Pose = m(new Pose(126.0, 57.0, Math.toRadians(0)));
    Pose shoot1Pose = m(new Pose(85.0, 77.0, Math.toRadians(320)));

    Pose openGate1Pose = m(new Pose(122, 66, Math.toRadians(340)));

    Pose pickUp2Pose = m(new Pose(132.0, 58.0, Math.toRadians(50)));
    Pose shoot2Pose = m(new Pose(85.0, 77.0, Math.toRadians(320)));

    Pose openGate2Pose = m(new Pose(122, 66, Math.toRadians(340)));

    Pose pickUp3Pose = m(new Pose(132.0, 58.0, Math.toRadians(50)));
    Pose shoot3Pose = m(new Pose(85.0, 77.0, Math.toRadians(320)));

    Pose openGate3Pose = m(new Pose(122, 66, Math.toRadians(340)));

    Pose pickUp4Pose = m(new Pose(132.0, 58.0, Math.toRadians(50)));

    Pose shoot4Pose = m(new Pose(89.0, 81.0, Math.toRadians(320)));
    Pose shoot4ControlPoint = m(new Pose(108, 62, Math.toRadians(320)));


    Pose pickUp5Pose = m(new Pose(125.0, 81.0, Math.toRadians(0)));

    Pose shoot5Pose = m(new Pose(92.0, 81.0, Math.toRadians(0)));
    Pose parkPose = m(new Pose(115.0, 81.0, Math.toRadians(0)));
    PathChain fullAuto;

    public void createPaths() {
        launchPreload = new Path(new BezierLine(startingPose, shootPreloadPose));
        launchPreload.setConstantHeadingInterpolation(startingPose.getHeading());

        intakeFirst = new Path(new BezierCurve(
                shootPreloadPose,
                pickUp1ControlPoint,
                pickUp1Pose));
        intakeFirst.setLinearHeadingInterpolation(
                shootPreloadPose.getHeading(),
                pickUp1Pose.getHeading());

        launchFirst = new Path(new BezierLine(
                pickUp1Pose,
                shoot1Pose));
        launchFirst.setTangentHeadingInterpolation();
        launchFirst.reverseHeadingInterpolation();

        openGate1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot1Pose,
                        openGate1Pose))
                .setConstantHeadingInterpolation(
                        openGate1Pose.getHeading())
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.7))
                .addPath(new BezierLine(
                        openGate1Pose,
                        pickUp2Pose)
                )
                .setLinearHeadingInterpolation(
                        openGate1Pose.getHeading(),
                        pickUp2Pose.getHeading())
                .build();

        launchSecond = new Path(new BezierLine(
                pickUp2Pose,
                shoot2Pose));
        launchSecond.setTangentHeadingInterpolation();
        launchSecond.reverseHeadingInterpolation();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot2Pose,
                        openGate2Pose))
                .setConstantHeadingInterpolation(
                        openGate2Pose.getHeading())
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.7))
                .addPath(new BezierLine(
                        openGate2Pose,
                        pickUp3Pose)
                )
                .setLinearHeadingInterpolation(
                        openGate2Pose.getHeading(),
                        pickUp3Pose.getHeading())
                .build();

        launchThird = new Path(new BezierLine(
                pickUp3Pose,
                shoot3Pose));
        launchThird.setTangentHeadingInterpolation();
        launchThird.reverseHeadingInterpolation();

        openGate3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot3Pose,
                        openGate3Pose))
                .setConstantHeadingInterpolation(
                        openGate3Pose.getHeading())
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.7))
                .addPath(new BezierLine(
                        openGate3Pose,
                        pickUp4Pose)
                )
                .setLinearHeadingInterpolation(
                        openGate2Pose.getHeading(),
                        pickUp3Pose.getHeading())
                .build();

        launchFourth = new Path(new BezierCurve(
                pickUp4Pose,
                shoot4ControlPoint,
                shoot4Pose));
        launchFourth.setTangentHeadingInterpolation();
        launchFourth.reverseHeadingInterpolation();

        intakeFive = new Path(new BezierLine(
                shoot4Pose,
                pickUp5Pose));
        intakeFive.setConstantHeadingInterpolation(pickUp5Pose.getHeading());

        launchFive = new Path(new BezierLine(
                pickUp5Pose,
                shoot5Pose));
        launchFive.setConstantHeadingInterpolation(pickUp5Pose.getHeading());

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

                        pedroSb.followPathCmd(openGate1),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new WaitCommand(600),

                        pedroSb.followPathCmd(launchSecond),

                        pedroSb.followPathCmd(openGate2),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new WaitCommand(600),

                        pedroSb.followPathCmd(launchThird),

                        pedroSb.followPathCmd(openGate3),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new WaitCommand(300),

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

