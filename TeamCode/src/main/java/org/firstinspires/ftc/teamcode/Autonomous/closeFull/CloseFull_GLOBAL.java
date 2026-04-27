package org.firstinspires.ftc.teamcode.Autonomous.closeFull;

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

public class CloseFull_GLOBAL extends OpModeCommand {

    public CloseFull_GLOBAL(Alliance alliance) {
        super(alliance, true, true);
    }

    private PathChain openGate1, openGate2;
    private Path launchPreload, intakeFirst, launchFirst, intakeSecond, launchSecond, intakeThird, launchThird, openGate3, intakeFourth, launchFourth, intakeFive, launchFive, park;

    private Pose currentStartingPose;
    Command autoCommand, parkCommand;

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

    Pose pickUp2Pose = m(new Pose(130.5, 58.0, Math.toRadians(50)));
    Pose shoot2Pose = m(new Pose(88.0, 81.0, Math.toRadians(0)));

    Pose openGate2ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate2Pose = m(new Pose(127, 63, Math.toRadians(0)));

    Pose pickUp3Pose = m(new Pose(131, 59.0, Math.toRadians(50)));
    Pose shoot3Pose = m(new Pose(88.0, 81.0, Math.toRadians(0)));

    Pose openGate3ControlPoint = m(new Pose(106.0, 62.0, Math.toRadians(0)));
    Pose openGate3Pose = m(new Pose(127, 63, Math.toRadians(0)));

    Pose pickUp4Pose = m(new Pose(131, 59.0, Math.toRadians(50)));
    Pose shoot4Pose = m(new Pose(88.0, 84.0, Math.toRadians(0)));
    Pose pickUp5Pose = m(new Pose(125.0, 84.0, Math.toRadians(0)));

    Pose shoot5Pose = m(new Pose(88, 108.0, Math.toRadians(0)));
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


        //SEBAS ERROR EN ESTE PATH, IGUAL SIGUE TENIENDO ERROR EN EL INTAKE FOURTH Y LAUNCH FOURTH
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
        launchFive.setTangentHeadingInterpolation();
        launchFive.reverseHeadingInterpolation();

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

                                pedroSb.followPathCmd(openGate1).withTimeout(1600),

                                new InstantCommand(() -> follower.setMaxPower(1)),

                                pedroSb.followPathCmd(intakeSecond).withTimeout(800),

                                new WaitCommand(800),

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

                                pedroSb.followPathCmd(openGate2).withTimeout(1800),

                                new InstantCommand(() -> follower.setMaxPower(1)),

                                new WaitCommand(100),

                                pedroSb.followPathCmd(intakeThird).withTimeout(800),

                                new WaitCommand(800),

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

                                pedroSb.followPathCmd(openGate3),

                                new WaitCommand(400),

                                pedroSb.followPathCmd(intakeFourth).withTimeout(800),

                                new WaitCommand(200),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                pedroSb.followPathCmd(launchFourth),
                                                shootThreeSpamerCloseCMD()

                                        ),
                                        new aimCMD(shooterSb, false, true)
                                ),

                                /// FOURTH_LAUNCHED

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

