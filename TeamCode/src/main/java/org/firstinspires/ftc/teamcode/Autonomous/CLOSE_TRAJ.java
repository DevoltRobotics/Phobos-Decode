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
        super(Alliance.RED, true, true);
    }

    private PathChain openGate1, openGate2;
    private Path launchPreload, intakeFirst, launchFirst, intakeSecond, launchSecond, intakeThird, launchThird, openGate3, intakeFourth, launchFourth, intakeFive, launchFive, park;

    private Pose currentStartingPose;
    Command autoCommand;

    Pose m(Pose p) {
        return Alliance.BLUE.equals(currentAlliance) ? p.mirror() : p;
    }

    Pose startingPose = m(new Pose(110.0, 134.5, Math.toRadians(0)));

    Pose shootPreloadPose = m(new Pose(87.0, 85.0, Math.toRadians(320)));
    Pose pickUp1ControlPoint = m(new Pose(88.0, 54.0, Math.toRadians(0)));
    Pose pickUp1Pose = m(new Pose(126.0, 57.0, Math.toRadians(0)));

    Pose shoot1ControlPoint = m(new Pose(105.0, 62.0, Math.toRadians(0)));
    Pose shoot1Pose = m(new Pose(88.0, 81.0, Math.toRadians(320)));

    Pose openGate1ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate1Pose = m(new Pose(127, 66, Math.toRadians(0)));

    Pose pickUp2Pose = m(new Pose(131.0, 59.0, Math.toRadians(50)));
    Pose shoot2Pose = m(new Pose(88.0, 81.0, Math.toRadians(0)));

    Pose openGate2ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate2Pose = m(new Pose(127, 63, Math.toRadians(0)));

    Pose pickUp3Pose = m(new Pose(131, 59.0, Math.toRadians(50)));
    Pose shoot3Pose = m(new Pose(88.0, 81.0, Math.toRadians(320)));

    Pose openGate3ControlPoint = m(new Pose(90, 62.0, Math.toRadians(0)));
    Pose openGate3Pose = m(new Pose(121, 71, Math.toRadians(270)));
    Pose pickUp4Pose = m(new Pose(134.0, 36, Math.toRadians(270)));
    Pose shoot4Pose = m(new Pose(88.0, 81.0, Math.toRadians(0)));
    Pose pickUp5Pose = m(new Pose(125.0, 84.0, Math.toRadians(0)));

    Pose shoot5Pose = m(new Pose(92.0, 85.0, Math.toRadians(0)));
    Pose parkPose = m(new Pose(115.0, 85.0, Math.toRadians(0)));

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
                .addPath(new BezierCurve(
                        shoot1Pose,
                        openGate1ControlPoint,
                        openGate1Pose))
                .setConstantHeadingInterpolation(
                        openGate1Pose.getHeading())
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.7))
                .build();

        intakeSecond = new Path(new BezierLine(
                openGate1Pose,
                pickUp2Pose));
        intakeSecond.setLinearHeadingInterpolation(
                openGate1Pose.getHeading(),
                pickUp2Pose.getHeading());

        launchSecond = new Path(new BezierLine(
                pickUp2Pose,
                shoot2Pose));
        launchSecond.setLinearHeadingInterpolation(
                pickUp2Pose.getHeading(),
                shoot2Pose.getHeading()
        );

        openGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shoot2Pose,
                        openGate2ControlPoint,
                        openGate2Pose))
                .setConstantHeadingInterpolation(
                        openGate2Pose.getHeading())
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.7))
                .build();


        intakeThird = new Path(new BezierLine(
                openGate2Pose,
                pickUp3Pose));
        intakeThird.setLinearHeadingInterpolation(
                openGate2Pose.getHeading(),
                pickUp3Pose.getHeading());

        launchThird = new Path(new BezierLine(
                pickUp3Pose,
                shoot3Pose));
        launchThird.setLinearHeadingInterpolation(
                pickUp3Pose.getHeading(),
                shoot3Pose.getHeading()
        );

        openGate3 = new Path(new BezierCurve(
                shoot3Pose,
                openGate3ControlPoint,
                openGate3Pose));
        openGate3.setConstantHeadingInterpolation(
                openGate3Pose.getHeading()
        );


        intakeFourth = new Path(new BezierLine(
                openGate3Pose,
                pickUp4Pose));
        intakeFourth.setConstantHeadingInterpolation(
                openGate3Pose.getHeading());

        launchFourth = new Path(new BezierLine(
                pickUp4Pose,
                shoot4Pose));
        launchFourth.setLinearHeadingInterpolation(
                pickUp4Pose.getHeading(),
                shoot4Pose.getHeading()
        );

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

                        pedroSb.followPathCmd(intakeSecond),

                        new WaitCommand(600),

                        pedroSb.followPathCmd(launchSecond),

                        pedroSb.followPathCmd(openGate2),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        pedroSb.followPathCmd(intakeThird),

                        new WaitCommand(600),

                        pedroSb.followPathCmd(launchThird),

                        pedroSb.followPathCmd(openGate3),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        pedroSb.followPathCmd(intakeFourth),

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

