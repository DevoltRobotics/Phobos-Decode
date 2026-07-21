package org.firstinspires.ftc.teamcode.Autonomous.Close_Partner;

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

public class ClosePartner_Global extends OpModeCommand {

    public ClosePartner_Global(Alliance alliance) {
        super(alliance, true, true);
    }

    private PathChain openGate1, openGate2, openGate3;
    private Path launchPreload, intakeFirst, launchFirst, launchSecond, launchThird, launchFourth, intakeFive, launchFive, park;

    private Pose currentStartingPose;
    Command autoCommand, parkCommand;

    Pose m(Pose p) {
        return Alliance.BLUE.equals(currentAlliance) ? p.mirror() : p;
    }
    private Pose startingPose;
    private Pose shootPreloadPose;

    private Pose pickUp1ControlPoint;
    private Pose pickUp1Pose;
    private Pose shoot1Pose;

    private Pose openGate1Pose;

    private Pose pickUp2Pose;
    private Pose pickUp2ControlPoint;
    private Pose shoot2Pose;

    private Pose openGate2Pose;

    private Pose pickUp3Pose;
    private Pose pickUp3ControlPoint;
    private Pose shoot3Pose;

    private Pose openGate3Pose;

    private Pose pickUp4Pose;
    private Pose pickUp4ControlPoint;

    private Pose shoot4Pose;
    private Pose shoot4ControlPoint;

    private Pose pickUp5Pose;

    private Pose shoot5Pose;
    private Pose parkPose;

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
                .setTimeoutConstraint(1)
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
                .addPath(new BezierCurve(
                        openGate2Pose,
                        pickUp3ControlPoint,
                        pickUp3Pose)
                )
                .setLinearHeadingInterpolation(
                        openGate2Pose.getHeading(),
                        pickUp3Pose.getHeading())
                .setTimeoutConstraint(1)
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
                .addPath(new BezierCurve(
                        openGate3Pose,
                        pickUp4ControlPoint,
                        pickUp4Pose)
                )
                .setLinearHeadingInterpolation(
                        openGate3Pose.getHeading(),
                        pickUp4Pose.getHeading())
                .setTimeoutConstraint(1)
                .build();

        launchFourth = new Path(new BezierCurve(
                pickUp4Pose,
                shoot4ControlPoint,
                shoot4Pose));
        launchFourth.setTangentHeadingInterpolation();
        launchFourth.reverseHeadingInterpolation();
        launchFourth.setTimeoutConstraint(1);

        intakeFive = new Path(new BezierLine(
                shoot4Pose,
                pickUp5Pose));
        intakeFive.setConstantHeadingInterpolation(pickUp5Pose.getHeading());

        launchFive = new Path(new BezierLine(
                pickUp5Pose,
                shoot5Pose));
        launchFive.setConstantHeadingInterpolation(pickUp5Pose.getHeading());
        launchFive.setTimeoutConstraint(1);

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

        openGate1Pose = m(new Pose(120, 65.5, Math.toRadians(340)));

        pickUp2Pose = m(new Pose(132, 57.5, Math.toRadians(40)));
        pickUp2ControlPoint = m(new Pose(124.0, 56.0, Math.toRadians(50)));

        shoot2Pose = m(new Pose(82.0, 75.0, Math.toRadians(320)));

        openGate2Pose = m(new Pose(120, 65.5, Math.toRadians(340)));

        pickUp3Pose = m(new Pose(132, 57.5, Math.toRadians(40)));
        pickUp3ControlPoint = m(new Pose(124.0, 64.0, Math.toRadians(50)));

        shoot3Pose = m(new Pose(85.0, 77.0, Math.toRadians(320)));

        openGate3Pose = m(new Pose(120, 65.5, Math.toRadians(340)));

        pickUp4Pose = m(new Pose(132, 57.5, Math.toRadians(40)));
        pickUp4ControlPoint = m(new Pose(124.0, 56.0, Math.toRadians(50)));

        shoot4Pose = m(new Pose(89.0, 81.0, Math.toRadians(320)));
        shoot4ControlPoint = m(new Pose(108, 62, Math.toRadians(320)));

        pickUp5Pose = m(new Pose(125.0, 81.0, Math.toRadians(0)));

        shoot5Pose = m(new Pose(97.0, 81.0, Math.toRadians(0)));
        parkPose = m(new Pose(115.0, 81.0, Math.toRadians(0)));

        follower.setStartingPose(startingPose);

        new SequentialCommandGroup(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                new WaitCommand(200),

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

                                        new aimCMD(shooterSb, false, true, -30)

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

                                        new aimCMD(shooterSb, false, true, -30)
                                ),

                                /// FIRST_LAUNCHED

                                stopShootCMD(false),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),
                                new InstantCommand(() -> follower.setMaxPower(0.6)),
                                pedroSb.followPathCmd(openGate1).withTimeout(1600),

                                new InstantCommand(() -> follower.setMaxPower(1)),

                                new WaitCommand(800),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchSecond),
                                                shootThreeSpamerCloseCMD()

                                        ),
                                        new aimCMD(shooterSb, false, true, -30)
                                ),

                                /// SECOND_LAUNCHED

                                stopShootCMD(false),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),
                                new InstantCommand(() -> follower.setMaxPower(0.6)),

                                pedroSb.followPathCmd(openGate2).withTimeout(1800),

                                new InstantCommand(() -> follower.setMaxPower(1)),

                                new WaitCommand(800),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchThird),
                                                shootThreeSpamerCloseCMD()

                                        ),
                                        new aimCMD(shooterSb, false, true, -30)
                                ),

                                ///THIRD_LAUNCHED

                                stopShootCMD(false),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                new InstantCommand(() -> follower.setMaxPower(0.6)),
                                pedroSb.followPathCmd(openGate3).withTimeout(1800),

                                new InstantCommand(() -> follower.setMaxPower(1)),

                                new WaitCommand(800),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchFourth),
                                                shootThreeSpamerCloseCMD()

                                        ),
                                        new aimCMD(shooterSb, false, true, -30)
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
                                        new aimCMD(shooterSb, false, true, -30)
                                )

                        ),
                        new WaitCommand(29000)
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

