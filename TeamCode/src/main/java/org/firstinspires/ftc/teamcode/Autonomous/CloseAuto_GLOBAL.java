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
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.detectMotifCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

public class CloseAuto_GLOBAL extends OpModeCommand {

    private Path launchPreload, park;
    private PathChain openGate, intakeFirst, launchFirst, prepareForIntakeSecond, intakeSecond, launchSecond, prepareForIntakeThird, intakeThird, launchThird;

    private Pose currentStartingPose;
    Command autoCommand;

    public static Pose startingPose = new Pose(123.0, 123.0, Math.toRadians(38));

    public static Pose shootPreloadPose = new Pose(86.0, 82.0, Math.toRadians(320));

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

    public static Pose shoot3ControlPoint = new Pose(115, 63, Math.toRadians(340));

    public static Pose shoot3Pose = new Pose(86.0, 81.0, Math.toRadians(340));

    public static Pose parkPose = new Pose(108.0, 70.0, Math.toRadians(0));

    public CloseAuto_GLOBAL(Aliance aliance) {
        super(aliance, true);
    }

    public void createPaths() {

        if (currentAliance == Aliance.RED) {

            launchPreload = new Path(new BezierLine(startingPose, shootPreloadPose));
            launchPreload.setLinearHeadingInterpolation(startingPose.getHeading(), shootPreloadPose.getHeading());

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
            launchPreload = new Path(new BezierLine(
                    startingPose.mirror(),
                    shootPreloadPose.mirror()));
            launchPreload.setLinearHeadingInterpolation(
                    startingPose.mirror().getHeading(),
                    shootPreloadPose.mirror().getHeading());


            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shootPreloadPose.mirror(),
                            pick1ControlPoint.mirror(),
                            pick1Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();


            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pick1Pose.mirror(),
                            openGateControlPoint.mirror(),
                            openGatePose.mirror()))
                    .setLinearHeadingInterpolation(
                            pick1Pose.mirror().getHeading(),
                            openGatePose.mirror().getHeading())
                    .build();


            launchFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            openGatePose.mirror(),
                            shoot1ControlPoint.mirror(),
                            shoot1Pose.mirror()))
                    .setLinearHeadingInterpolation(
                            openGatePose.mirror().getHeading(),
                            shoot1Pose.mirror().getHeading())
                    .build();


            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot1Pose.mirror(),
                            pickUp2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            shoot1Pose.mirror().getHeading())
                    .build();


            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp2Pose.mirror(),
                            shoot2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pickUp2Pose.mirror().getHeading())
                    .build();


            intakeThird = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot2Pose.mirror(),
                            pickUp2ControlPoint.mirror(),
                            pickUp3Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();


            launchThird = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pickUp3Pose.mirror(),
                            shoot3ControlPoint.mirror(),
                            shoot3Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();


            park = new Path(new BezierLine(
                    shoot3Pose.mirror(),
                    parkPose.mirror()));
            park.setLinearHeadingInterpolation(
                    shoot3Pose.mirror().getHeading(),
                    parkPose.mirror().getHeading());
        }
    }

    @Override
    public void initialize() {

        if (currentAliance.equals(Aliance.RED)) {
            follower.setStartingPose(startingPoseCloseRed);

        } else {
            follower.setStartingPose(startingPoseCloseBlue);

        }

        new SequentialCommandGroup(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                new rampCMD(sorterSb, upRampPos),

                new WaitCommand(200),

                new lateralBlockersCMD(sorterSb, 0, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos)

        ).schedule();

        createPaths();
        autoCommand =
                new SequentialCommandGroup(

                        new shooterToVelCMD(shooterSb, 1300),

                        new turretToPosCMD(turretSb, 80.0),

                        pedroSb.followPathCmd(launchPreload),

                        new WaitCommand(500),

                        shootThreeSpamerCMD(1300),

                        ///PRELOAD_LAUNCHED

                        stopShootCMD(false),

                        new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                        pedroSb.followPathCmd(intakeFirst).withTimeout(2000),

                        new WaitCommand(300),

                        pedroSb.followPathCmd(openGate).withTimeout(800),

                        new WaitCommand(300),

                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(launchFirst).withTimeout(2300),
                                new turretToBasketCMD(turretSb, visionSb),
                                new shooterToBasketCMD(shooterSb, visionSb, 1300)
                        ),

                        shootThreeSpamerCMD(1300),

                        /// FIRST_LAUNCHED

                        stopShootCMD(false),

                        new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                        pedroSb.followPathCmd(intakeSecond).withTimeout(2000),

                        new WaitCommand(300),

                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(launchSecond).withTimeout(2300),
                                new turretToBasketCMD(turretSb, visionSb),
                                new shooterToBasketCMD(shooterSb, visionSb, 1300)
                        ),

                        shootThreeSpamerCMD(1300),

                        /// SECOND_LAUNCHED

                        stopShootCMD(true),
                        new moveIntakeAutonomousCMD(intakeSb, 1),

                        pedroSb.followPathCmd(intakeThird).withTimeout(2000),

                        new WaitCommand(300),

                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(launchThird).withTimeout(2300),
                                new turretToBasketCMD(turretSb, visionSb),
                                new shooterToBasketCMD(shooterSb, visionSb, 1300)
                        ),

                        shootThreeSpamerCMD(1300),

                        /// THIRD_LAUNCHED

                        stopShootCMD(false),
                        pedroSb.followPathCmd(park),

                        new WaitCommand(300)

                );
    }

    @Override
    public void start() {
        autoCommand.schedule();

    }

}

