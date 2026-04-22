package org.firstinspires.ftc.teamcode.Autonomous.closeFull;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHSortingPos;
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
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.aimCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;

public class CloseFull_GLOBAL extends OpModeCommand {

    public CloseFull_GLOBAL(Alliance alliance) {
        super(alliance, true);
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
    Pose openGate1Pose = m(new Pose(125.0, 64.0, Math.toRadians(0)));

    Pose pickUp2Pose = m(new Pose(130.0, 53.0, Math.toRadians(40)));
    Pose shoot2Pose = m(new Pose(88.0, 81.0, Math.toRadians(320)));

    Pose openGate2ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate2Pose = m(new Pose(125.0, 64.0, Math.toRadians(0)));

    Pose pickUp3Pose = m(new Pose(131.0, 53.0, Math.toRadians(40)));
    Pose shoot3Pose = m(new Pose(88.0, 81.0, Math.toRadians(320)));

    Pose openGate3ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate3Pose = m(new Pose(125.0, 64.0, Math.toRadians(0)));

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
                .setTimeoutConstraint(1)
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
                .setTimeoutConstraint(1)
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
                .setTimeoutConstraint(0)
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
                .setTimeoutConstraint(0)
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

        new SequentialCommandGroup(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                new rampCMD(sorterSb, upRampPos),

                new WaitCommand(200),

                //new turretToPosCMD(turretSb, 0.0),

                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos)


        ).schedule();

        createPaths();


        autoCommand = new SequentialCommandGroup(

                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intakeSb.setIntakePower(0.3, 0)),

                                pedroSb.followPathCmd(launchPreload).withTimeout(1400),

                                new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHFreePos)),

                                new WaitCommand(1000),

                                new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),

                                new WaitCommand(950)

                                ),

                        new aimCMD(shooterSb, false, true)

                ),

                ///PRELOAD_LAUNCHED

                stopShootCMD(false),

                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                pedroSb.followPathCmd(intakeFirst).withTimeout(2000),

                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                pedroSb.followPathCmd(launchFirst).withTimeout(2300),
                                shootThreeSpamerCloseCMD()
                        ),

                        new aimCMD(shooterSb, false, true)
                ),

                /// FIRST_LAUNCHED

                stopShootCMD(false),

                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(openGate1).withTimeout(2200),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> pedroSb.follower.setMaxPower(0.6))
                        )),

                new InstantCommand(() -> follower.setMaxPower(1)),

                pedroSb.followPathCmd(intakeSecond).withTimeout(800),

                new WaitCommand(800),

                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                pedroSb.followPathCmd(launchSecond).withTimeout(2300),
                                shootThreeSpamerCloseCMD()

                        ),
                        new aimCMD(shooterSb, false, true)
                ),

                /// SECOND_LAUNCHED

                stopShootCMD(false),

                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(openGate2).withTimeout(2200),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> pedroSb.follower.setMaxPower(0.6))
                        )),

                new InstantCommand(() -> follower.setMaxPower(1)),

                pedroSb.followPathCmd(intakeThird).withTimeout(800),

                new WaitCommand(800),

                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                pedroSb.followPathCmd(launchThird).withTimeout(2300),
                                shootThreeSpamerCloseCMD()

                        ),
                        new aimCMD(shooterSb, false, true)
                ),

                ///THIRD_LAUNCHED
                stopShootCMD(false),

                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(openGate3).withTimeout(2200),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new InstantCommand(() -> pedroSb.follower.setMaxPower(0.6))
                        )),
                new WaitCommand(200),

                new InstantCommand(() -> follower.setMaxPower(1)),

                pedroSb.followPathCmd(intakeFourth).withTimeout(800),

                new WaitCommand(800),

                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                pedroSb.followPathCmd(launchFourth).withTimeout(2300),
                                shootThreeSpamerCloseCMD()

                        ),
                        new aimCMD(shooterSb, false, true)
                ),

                /// FOURTH_LAUNCHED

                stopShootCMD(false),

                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                new InstantCommand(() -> follower.setMaxPower(1)),

                pedroSb.followPathCmd(intakeFive).withTimeout(1200),

                new InstantCommand(() -> follower.setMaxPower(1)),

                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                pedroSb.followPathCmd(launchFive).withTimeout(2300),
                                shootThreeSpamerCloseCMD()

                        ),
                        new aimCMD(shooterSb, false, true)
                ),

                /// FIVE_LAUNCHED

                stopShootCMD(false),

                new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),
                new InstantCommand(() -> shooterSb.setShooterTarget(0)),

                pedroSb.followPathCmd(park)
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

    }
}

