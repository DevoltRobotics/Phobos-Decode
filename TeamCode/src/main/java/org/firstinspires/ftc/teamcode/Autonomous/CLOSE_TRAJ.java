package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseBlue;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseRed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.detectMotifCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Autonomous
public class CLOSE_TRAJ extends OpModeCommand {

    private Path launchPreload, park;
    private PathChain openGate, intakeFirst, launchFirst, intakeSecond, launchSecond, prepareForIntakeThird, intakeThird, launchThird;

    private Pose currentStartingPose;
    Command autoCommand;

    public static Pose startingPose = new Pose(123.0, 123.0, Math.toRadians(38));

    public static Pose shootPreloadPose = new Pose(86.0, 82.0, Math.toRadians(38));

    public static Pose pick1ControlPoint = new Pose(96, 58, Math.toRadians(0));

    public static Pose pick1Pose = new Pose(136.0, 58.0, Math.toRadians(0));

    public static Pose openGateControlPoint = new Pose(118.0, 60.0, Math.toRadians(0));

    public static Pose openGatePose = new Pose(128.0, 68.0, Math.toRadians(270));

    public static Pose shoot1ControlPoint = new Pose(91.0, 66.0, Math.toRadians(0));
    public static Pose shoot1Pose = new Pose(88.0, 85.0, Math.toRadians(0));

    public static Pose pickUp2Pose = new Pose(128.0, 85.0, Math.toRadians(0));

    public static Pose shoot2Pose = new Pose(88.0, 85.0, Math.toRadians(0));

    public static Pose pickUp2ControlPoint = new Pose(88.0, 30.0, Math.toRadians(0));

    public static Pose pickUp3Pose = new Pose(135.0, 35.0, Math.toRadians(0));

    public static Pose shoot3ControlPoint = new Pose(105.0, 29.0, Math.toRadians(0));

    public static Pose shoot3Pose = new Pose(86.0, 81.0, Math.toRadians(0));

    public static Pose parkPose = new Pose(108.0, 70.0, Math.toRadians(0));

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


            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pick1Pose,
                            openGateControlPoint,
                            openGatePose))
                    .setLinearHeadingInterpolation(
                            pick1Pose.getHeading(),
                            openGatePose.getHeading())
                    .build();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            openGatePose,
                            shoot1ControlPoint,
                            shoot1Pose))
                    .setLinearHeadingInterpolation(
                            openGatePose.getHeading(),
                            shoot1Pose.getHeading())
                    .build();

            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot1Pose,
                            pickUp2Pose))
                    .setConstantHeadingInterpolation(
                            shoot1Pose.getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp2Pose,
                            shoot2Pose))
                    .setConstantHeadingInterpolation(
                            pickUp2Pose.getHeading())
                    .build();

            intakeThird = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot2Pose,
                            pickUp2ControlPoint,
                            pickUp3Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            launchThird = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pickUp3Pose,
                            shoot3ControlPoint,
                            shoot3Pose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            park = new Path(new BezierLine(shoot3Pose, parkPose));
            park.setLinearHeadingInterpolation(shoot3Pose.getHeading(), parkPose.getHeading());


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
                autoCommand =
                        new SequentialCommandGroup(

                                pedroSb.followPathCmd(launchPreload),

                                pedroSb.followPathCmd(intakeFirst),

                                pedroSb.followPathCmd(openGate),

                                pedroSb.followPathCmd(launchFirst),
                                pedroSb.followPathCmd(intakeSecond),

                                pedroSb.followPathCmd(launchSecond),

                                pedroSb.followPathCmd(intakeThird),

                                pedroSb.followPathCmd(launchThird),
                                pedroSb.followPathCmd(park)


                );
    }

    @Override
    public void start() {
        autoCommand.schedule();

    }

}

