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
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

public class CloseFull_GLOBAL extends OpModeCommand {

    private Path launchPreload, park;
    private PathChain intakeFirst, launchFirst, openGate1, intakeSecond, launchSecond, intakeThird, launchThird, intakeFourth, launchFourth;

    private Pose currentStartingPose;
    Command autoCommand;

    static Pose startingPose = new Pose(123.0, 123.0, Math.toRadians(38));

    static Pose shootPreloadPose = new Pose(88.0, 84.0, Math.toRadians(38));

    static Pose pick1ControlPoint = new Pose(92, 60, Math.toRadians(0));

    static Pose pick1Pose = new Pose(131.5, 58.0, Math.toRadians(0));

    static Pose shoot1ControlPoint = new Pose(91.0, 66.0, Math.toRadians(0));
    static Pose shoot1Pose = new Pose(85.0, 76.0, Math.toRadians(330));

    static Pose opengate1ControlPoint = new Pose(100, 64, Math.toRadians(0));
    static Pose opengate1Pose = new Pose(126, 66.0, Math.toRadians(0));

    static Pose pick2ControlPoint = new Pose(125, 52, Math.toRadians(20));
    static Pose pick2Pose = new Pose(135.0, 51.0, Math.toRadians(80));

    static Pose shoot2ControlPoint = new Pose(99, 62.0, Math.toRadians(0));

    static Pose shoot2Pose = new Pose(88.0, 86.0, Math.toRadians(0));

    static Pose pickUp3Pose = new Pose(127.0, 86.0, Math.toRadians(0));

    static Pose shoot3Pose = new Pose(88.0, 84.0, Math.toRadians(0));

    static Pose pickUp4ControlPoint = new Pose(88.0, 30.0, Math.toRadians(0));

    static Pose pickUp4Pose = new Pose(134.0, 35.0, Math.toRadians(0));

    static Pose shoot4ControlPoint = new Pose(115, 63, Math.toRadians(340));

    static Pose shoot4Pose = new Pose(87.0, 80.0, Math.toRadians(330));
    static Pose parkPose = new Pose(108.0, 70.0, Math.toRadians(0));

    public CloseFull_GLOBAL(Aliance aliance) {
        super(aliance, true);
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
                    .setTimeoutConstraint(1)
                    .build();

            openGate1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot1Pose,
                            opengate1ControlPoint,
                            opengate1Pose))
                    .setTangentHeadingInterpolation()
                    .setTimeoutConstraint(1)
                    .build();


            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            opengate1Pose,
                            pick2ControlPoint,
                            pick2Pose))
                    .setLinearHeadingInterpolation(
                            opengate1Pose.getHeading(),
                            pick2Pose.getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pick2Pose,
                            shoot2ControlPoint,
                            shoot2Pose))
                    .setLinearHeadingInterpolation(
                            pick2Pose.getHeading(),
                            shoot2Pose.getHeading()
                    )
                    .setTimeoutConstraint(1)
                    .build();


            intakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot2Pose,
                            pickUp3Pose))
                    .setConstantHeadingInterpolation(
                            pickUp3Pose.getHeading()
                    )
                    .build();

            launchThird = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp3Pose,
                            shoot3Pose))
                    .setConstantHeadingInterpolation(
                            pickUp3Pose.getHeading()
                    )
                    .setTimeoutConstraint(1)
                    .build();

            intakeFourth = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot3Pose,
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
                    .setTimeoutConstraint(1)
                    .build();

            park = new Path(new BezierLine(shoot4Pose, parkPose));
            park.setLinearHeadingInterpolation(shoot4Pose.getHeading(), parkPose.getHeading());

        } else {
            launchPreload = new Path(new BezierLine(
                    startingPose.mirror(),
                    shootPreloadPose.mirror()));
            launchPreload.setConstantHeadingInterpolation(
                    startingPose.mirror().getHeading());


            intakeFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shootPreloadPose.mirror(),
                            pick1ControlPoint.mirror(),
                            pick1Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();


            launchFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pick1Pose.mirror(),
                            shoot1ControlPoint.mirror(),
                            shoot1Pose.mirror()))
                    .setLinearHeadingInterpolation(
                            pick1Pose.mirror().getHeading(),
                            shoot1Pose.mirror().getHeading())
                    .setTimeoutConstraint(1)
                    .build();


            openGate1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot1Pose.mirror(),
                            opengate1ControlPoint.mirror(),
                            opengate1Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .setTimeoutConstraint(1)
                    .build();


            intakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            opengate1Pose.mirror(),
                            pick2Pose.mirror()))
                    .setLinearHeadingInterpolation(
                            opengate1Pose.mirror().getHeading(),
                            pick2Pose.mirror().getHeading())
                    .build();


            launchSecond = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pick2Pose.mirror(),
                            shoot2ControlPoint.mirror(),
                            shoot2Pose.mirror()))
                    .setLinearHeadingInterpolation(
                            pick2Pose.mirror().getHeading(),
                            shoot2Pose.mirror().getHeading())
                    .setTimeoutConstraint(1)
                    .build();


            intakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot2Pose.mirror(),
                            pickUp3Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pickUp3Pose.mirror().getHeading())
                    .build();


            launchThird = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp3Pose.mirror(),
                            shoot3Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pickUp3Pose.mirror().getHeading())
                    .setTimeoutConstraint(1)
                    .build();


            intakeFourth = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot3Pose.mirror(),
                            pickUp4ControlPoint.mirror(),
                            pickUp4Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();


            launchFourth = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pickUp4Pose.mirror(),
                            shoot4ControlPoint.mirror(),
                            shoot4Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .setTimeoutConstraint(1)
                    .build();


            park = new Path(new BezierLine(
                    shoot4Pose.mirror(),
                    parkPose.mirror()));
            park.setLinearHeadingInterpolation(
                    shoot4Pose.mirror().getHeading(),
                    parkPose.mirror().getHeading());

        }
    }

    @Override
    public void initialize() {

        if (currentAliance.equals(Aliance.RED)) {
            follower.setStartingPose(startingPose);

        } else {
            follower.setStartingPose(startingPose.mirror());

        }

        new SequentialCommandGroup(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                new rampCMD(sorterSb, upRampPos),

                new WaitCommand(200),

                new turretToPosCMD(turretSb, 0.0),

                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos)


        ).schedule();

        createPaths();

        autoCommand = new SequentialCommandGroup(

                new shooterToVelCMD(shooterSb, 1220),

                new moveIntakeAutonomousCMD(intakeSb, 0.3, 0),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -18.0),
                        new turretToPosCMD(turretSb, 18.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                pedroSb.followPathCmd(launchPreload).withTimeout(1400),

                new ParallelDeadlineGroup(
                        new WaitCommand(1350),

                        new SequentialCommandGroup(

                                new WaitCommand(300),

                                new moveIntakeAutonomousCMD(intakeSb, 1),
                                new horizontalBlockerCMD(sorterSb, blockerHFreePos)

                        ),

                        new turretToBasketCMD(turretSb, visionSb),
                        new shooterToBasketCMD(shooterSb, visionSb, turretSb)
                ),

                ///PRELOAD_LAUNCHED

                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                pedroSb.followPathCmd(intakeFirst).withTimeout(2000),

                new shooterToVelCMD(shooterSb, 1220),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -76.0),
                        new turretToPosCMD(turretSb, 76.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                pedroSb.followPathCmd(launchFirst).withTimeout(2300),

                shootThreeSpamerCloseCMD(),

                /// FIRST_LAUNCHED

                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                new InstantCommand(

                        () -> follower.setMaxPower(0.75)
                ),

                pedroSb.followPathCmd(openGate1).withTimeout(3000),

                new InstantCommand(

                        () -> follower.setMaxPower(1)
                ),
                pedroSb.followPathCmd(intakeSecond).withTimeout(800),

                new shooterToVelCMD(shooterSb, 1250),

                new WaitCommand(900),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -45.0),
                        new turretToPosCMD(turretSb, 45.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(launchSecond).withTimeout(2300),

                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new moveIntakeAutonomousCMD(intakeSb, -0.2, 0.8),
                                new WaitCommand(100),
                                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8)

                        )),
                shootThreeSpamerCloseCMD(),

                /// SECOND_LAUNCHED

                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                pedroSb.followPathCmd(intakeThird).withTimeout(1200),

                new shooterToVelCMD(shooterSb, 1250),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -48.0),
                        new turretToPosCMD(turretSb, 48.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(launchThird).withTimeout(2300),

                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new moveIntakeAutonomousCMD(intakeSb, -0.2, 0.8),
                                new WaitCommand(100),
                                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8)

                        )),
                shootThreeSpamerCloseCMD(),

                /// THIRD_LAUNCHED


                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                pedroSb.followPathCmd(intakeFourth).withTimeout(2300),

                new shooterToVelCMD(shooterSb, 1250),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -77.0),
                        new turretToPosCMD(turretSb, 77.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),


                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(launchFourth).withTimeout(2300),

                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new moveIntakeAutonomousCMD(intakeSb, -0.2, 0.8),
                                new WaitCommand(100),
                                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8)

                        )),

                shootThreeSpamerCloseCMD(),

                /// FOURTH_LAUNCHED

                stopShootCMD(false),
                pedroSb.followPathCmd(park),

                new WaitCommand(400)

        );
    }

    @Override
    public void start() {
        autoCommand.schedule();

    }

}

