package org.firstinspires.ftc.teamcode.Autonomous.Close_Full;

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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.aimCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;

public class CloseFull_Global extends OpModeCommand {

    public CloseFull_Global(Alliance alliance) {
        super(alliance, true, true);
    }

    private PathChain openGate1, openGate3;
    private Path launchPreload, intakeFirst, launchFirst, launchSecond, intakeThird, launchThird, launchFourth, intakeFive, launchFive, park;

    private Pose currentStartingPose;
    Command autoCommand, parkCommand;

    Pose m(Pose p) {
        return Alliance.BLUE.equals(currentAlliance) ? p.mirror() : p;
    }
    Pose startingPose;

    Pose shootPreloadPose;

    Pose pickUp1ControlPoint;
    Pose pickUp1Pose;
    Pose shoot1Pose;

    Pose openGate1Pose;

    Pose pickUp2Pose;
    Pose pickUp2ControlPoint;
    Pose shoot2Pose;

    Pose pickUp3Pose;
    Pose pickUp3PControlPoint;

    Pose shoot3Pose;

    Pose openGate2Pose;

    Pose pickUp4Pose;
    Pose pickUp4ControlPoint;

    Pose shoot4Pose;
    Pose shoot4ControlPoint;


    Pose pickUp5Pose;

    Pose shoot5Pose;
    Pose parkPose;

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
                .addPath(new BezierCurve(
                        openGate1Pose,
                        pickUp2ControlPoint,
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

        intakeThird = new Path(new BezierCurve(
                shoot2Pose,
                pickUp3PControlPoint,
                pickUp3Pose));
        intakeThird.setTangentHeadingInterpolation();

        launchThird = new Path(new BezierLine(
                pickUp3Pose,
                shoot3Pose));
        launchThird.setTangentHeadingInterpolation();
        launchThird.reverseHeadingInterpolation();

        openGate3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot3Pose,
                        openGate2Pose))
                .setConstantHeadingInterpolation(
                        openGate2Pose.getHeading())
                .addPath(new BezierCurve(
                        openGate2Pose,
                        pickUp4ControlPoint,
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
        startingPose = m(new Pose(110.0, 133, Math.toRadians(0)));

        shootPreloadPose = m(new Pose(87.0, 85.0, Math.toRadians(320)));

        pickUp1Pose = m(new Pose(126.0, 57.0, Math.toRadians(0)));
        pickUp1ControlPoint = m(new Pose(88.0, 54.0, Math.toRadians(0)));

        shoot1Pose = m(new Pose(85.0, 72.0, Math.toRadians(320)));

        openGate1Pose = m(new Pose(120.5, 65.5, Math.toRadians(360)));

        pickUp2Pose = m(new Pose(132, 57, Math.toRadians(45)));
        pickUp2ControlPoint = m(new Pose(124.0, 56.0, Math.toRadians(50)));

        shoot2Pose = m(new Pose(82.0, 75.0, Math.toRadians(320)));

        pickUp3Pose = m(new Pose(130, 35, Math.toRadians(0)));
        pickUp3PControlPoint = m(new Pose(90, 32, Math.toRadians(0)));

        shoot3Pose = m(new Pose(83.0, 72.0, Math.toRadians(320)));

        openGate2Pose = m(new Pose(120, 64, Math.toRadians(360)));
        pickUp4Pose = m(new Pose(132, 57, Math.toRadians(45)));
        pickUp4ControlPoint = m(new Pose(124.0, 56.0, Math.toRadians(50)));

        shoot4Pose = m(new Pose(89.0, 81.0, Math.toRadians(320)));
        shoot4ControlPoint = m(new Pose(108, 62, Math.toRadians(320)));

        pickUp5Pose = m(new Pose(125.0, 81.0, Math.toRadians(0)));

        shoot5Pose = m(new Pose(97.0, 81.0, Math.toRadians(0)));
        parkPose = m(new Pose(109.0, 81.0, Math.toRadians(0)));

        follower.setStartingPose(startingPose);

        new SequentialCommandGroup(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                new WaitCommand(200),

                //new turretToPosCMD(turretSb, 0.0),

                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos)


        ).schedule();

        createPaths();

        parkCommand = new SequentialCommandGroup(
                stopShootCMD(false),
                new InstantCommand(() -> shooterSb.setShooterTarget(0)),
                new InstantCommand(() -> follower.setMaxPower(1)),

                pedroSb.followPathCmd(park)
        );
        autoCommand = new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new SequentialCommandGroup(

                                new InstantCommand(() -> follower.setMaxPower(1)),

                                new rampCMD(sorterSb, upRampPos),
                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> intakeSb.setIntakePower(0.3, 0)),

                                                pedroSb.followPathCmd(launchPreload),

                                                new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHFreePos)),

                                                new WaitCommand(600),

                                                new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),

                                                new WaitCommand(950)
                                        ),

                                        new aimCMD(shooterSb, false, true)

                                ),

                                ///PRELOAD_LAUNCHED

                                stopShootCMD(false),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                pedroSb.followPathCmd(intakeFirst).withTimeout(1800),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchFirst),
                                                shootThreeSpamerCloseCMD()
                                        ),

                                        new aimCMD(shooterSb, false, true)
                                ),

                                /// FIRST_LAUNCHED

                                stopShootCMD(false),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),
                                new InstantCommand(() -> follower.setMaxPower(0.5)),
                                pedroSb.followPathCmd(openGate1).withTimeout(1600),

                                new InstantCommand(() -> follower.setMaxPower(1)),

                                new WaitCommand(850),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchSecond),
                                                shootThreeSpamerCloseCMD()

                                        ),
                                        new aimCMD(shooterSb, false, true)
                                ),

                                /// SECOND_LAUNCHED

                                stopShootCMD(false),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                pedroSb.followPathCmd(intakeThird).withTimeout(1800),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchThird),
                                                shootThreeSpamerCloseCMD()

                                        ),
                                        new aimCMD(shooterSb, false, true)
                                ),

                                ///THIRD_LAUNCHED

                                stopShootCMD(false),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                new InstantCommand(() -> follower.setMaxPower(0.5)),
                                pedroSb.followPathCmd(openGate3).withTimeout(1800),

                                new InstantCommand(() -> follower.setMaxPower(1)),

                                new WaitCommand(850),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchFourth),
                                                shootThreeSpamerCloseCMD()

                                        ),
                                        new aimCMD(shooterSb, false, true)
                                ),

                                ///FOURTH_LAUNCHED

                                stopShootCMD(false),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                pedroSb.followPathCmd(intakeFive).withTimeout(1200),


                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchFive),
                                                shootThreeSpamerCloseCMD()

                                        ),
                                        new aimCMD(shooterSb, false, true)
                                )

                        ),
                        new WaitCommand(29200)
                ),

                parkCommand
        );

    }

    @Override
    public void start() {
        if (autoCommand != null) {
            // Programamos el auton en el scheduler
            schedule(autoCommand);
        }

    }

    @Override
    public void run() {
        PedroSubsystem.EndPose = pedroSb.follower.getPose();

        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("EndPose", PedroSubsystem.EndPose);
    }
}

