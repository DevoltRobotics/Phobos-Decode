package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseBlue;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseRed;

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

@Autonomous
public class CLOSE_TRAJ extends OpModeCommand {

    private Path launchPreload, park;
    private PathChain openGate, intakeFirst, launchFirst, intakeSecond, launchSecond, intakeThird, launchThird, intakeFourth, launchFourth, intakeFive, launchFive;

    private Pose currentStartingPose;
    Command autoCommand;

    static Pose startingPose = new Pose(123.0, 123.0, Math.toRadians(38));

     static Pose shootPreloadPose = new Pose(88.0, 84.0, Math.toRadians(38));

     static Pose pick1ControlPoint = new Pose(95, 58, Math.toRadians(0));

     static Pose pick1Pose = new Pose(134.0, 58.0, Math.toRadians(0));

     static Pose shoot1ControlPoint = new Pose(91.0, 66.0, Math.toRadians(0));
     static Pose shoot1Pose = new Pose(88.0, 85.0, Math.toRadians(320));

     static Pose pickUp2Pose = new Pose(132.0, 57.0, Math.toRadians(40));

     static Pose shoot2Pose = new Pose(88.0, 85.0, Math.toRadians(0));

     static Pose pickUp3Pose = new Pose(132.0, 57.0, Math.toRadians(40));

     static Pose shoot3Pose = new Pose(88.0, 85.0, Math.toRadians(0));

     static Pose pickUp5Pose = new Pose(128.0, 85.0, Math.toRadians(0));

     static Pose shoot5Pose = new Pose(88.0, 85.0, Math.toRadians(0));

     static Pose pickUp4ControlPoint = new Pose(88.0, 30.0, Math.toRadians(0));

     static Pose pickUp4Pose = new Pose(135.0, 35.0, Math.toRadians(0));

     static Pose shoot4ControlPoint = new Pose(115, 63, Math.toRadians(340));

     static Pose shoot4Pose = new Pose(86.0, 81.0, Math.toRadians(340));

     static Pose parkPose = new Pose(108.0, 70.0, Math.toRadians(0));

    public CLOSE_TRAJ() {
        super(Aliance.RED, true);
    }

    public void createPaths() {

        if (currentAliance == Aliance.RED) {

            launchPreload = new Path(new BezierLine(startingPose, shootPreloadPose));
            launchPreload.setConstantHeadingInterpolation(startingPose.getHeading());

            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shootPreloadPose,
                            pick1ControlPoint,
                            pick1Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pick1Pose,
                            shoot1ControlPoint,
                            shoot1Pose))
                    .setLinearHeadingInterpolation(
                            pick1Pose.getHeading(),
                            shoot1Pose.getHeading())
                    .build();

            /*openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot1Pose,
                            openGateControlPoint,
                            openGatePose))
                    .setTangentHeadingInterpolation()
                    .build();

             */

            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot1Pose,
                            pickUp2Pose))
                    .setLinearHeadingInterpolation(
                            shoot1Pose.getHeading(),
                            pickUp2Pose.getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp2Pose,
                            shoot2Pose))
                    .setLinearHeadingInterpolation(
                            pickUp2Pose.getHeading(),
                            shoot2Pose.getHeading()
                    )
                    .build();



            intakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot2Pose,
                            pickUp3Pose))
                    .setLinearHeadingInterpolation(
                            shoot2Pose.getHeading(),
                            pickUp3Pose.getHeading())
                    .build();

            launchThird = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp3Pose,
                            shoot3Pose))
                    .setLinearHeadingInterpolation(
                            pickUp3Pose.getHeading(),
                            shoot3Pose.getHeading()
                    )
                    .build();

            intakeFourth = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot5Pose,
                            pickUp4ControlPoint,
                            pickUp4Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            launchFourth = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pickUp4Pose,
                            shoot4ControlPoint,
                            shoot4Pose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeFive = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot3Pose,
                            pickUp5Pose))
                    .setConstantHeadingInterpolation(
                            pickUp5Pose.getHeading())
                    .build();

            launchFive = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp5Pose,
                            shoot5Pose))
                    .setConstantHeadingInterpolation(
                            pickUp5Pose.getHeading())
                    .build();

            park = new Path(new BezierLine(shoot5Pose, parkPose));
            park.setLinearHeadingInterpolation(shoot5Pose.getHeading(), parkPose.getHeading());


        } else {

            /*

            launchPreload = new Path(new BezierLine(startingPoseCloseBlue, launchPreloadCloseBluePose));
            launchPreload.setConstantHeadingInterpolation(startingPoseCloseBlue.getHeading());

            prepareForIntakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(launchPreloadCloseBluePose, prepareForIntakeFirstCloseBluePose))
                    .setLinearHeadingInterpolation(launchPreloadCloseBluePose.getHeading(), prepareForIntakeFirstCloseBluePose.getHeading())
                    .build();

            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierLine(prepareForIntakeFirstCloseBluePose, intakeFirstCloseBluePose))
                    .setConstantHeadingInterpolation(intakeFirstCloseBluePose.getHeading())
                    .build();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(intakeFirstCloseBluePose, launchFirstCloseBluePose))
                    .setLinearHeadingInterpolation(intakeFirstCloseBluePose.getHeading(), launchFirstCloseBluePose.getHeading())
                    .build();

            prepareForIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(launchFirstCloseBluePose, prepareForIntakeSecondCloseBluePose))
                    .setLinearHeadingInterpolation(launchFirstCloseBluePose.getHeading(), prepareForIntakeSecondCloseBluePose.getHeading())
                    .build();

            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(prepareForIntakeSecondCloseBluePose, intakeSecondCloseBluePose))
                    .setConstantHeadingInterpolation(prepareForIntakeSecondCloseBluePose.getHeading())
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

            park = new Path(new BezierLine(launchSecondCloseBluePose, parkCloseBluePose));
            park.setLinearHeadingInterpolation(launchSecondCloseBluePose.getHeading(), parkCloseBluePose.getHeading());


             */
        }
    }

    @Override
    public void initialize() {

        if (currentAliance.equals(Aliance.RED)) {
            follower.setStartingPose(startingPoseCloseRed);

        } else {
            follower.setStartingPose(startingPoseCloseBlue);

        }

        createPaths();

                autoCommand =
                        new SequentialCommandGroup(

                                pedroSb.followPathCmd(launchPreload),

                                pedroSb.followPathCmd(intakeFirst),

                                pedroSb.followPathCmd(launchFirst),

                               // pedroSb.followPathCmd(openGate),

                                pedroSb.followPathCmd(intakeSecond),

                                pedroSb.followPathCmd(launchSecond),

                                pedroSb.followPathCmd(intakeThird),

                                pedroSb.followPathCmd(launchThird),

                                pedroSb.followPathCmd(intakeFourth),

                                pedroSb.followPathCmd(launchFourth),

                                pedroSb.followPathCmd(intakeFive),

                                pedroSb.followPathCmd(launchFive),

                                pedroSb.followPathCmd(park)




                );
    }

    @Override
    public void start() {
        autoCommand.schedule();

    }

}

