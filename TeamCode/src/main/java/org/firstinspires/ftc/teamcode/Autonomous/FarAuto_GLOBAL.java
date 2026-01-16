package org.firstinspires.ftc.teamcode.Autonomous;

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
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

public class FarAuto_GLOBAL extends OpModeCommand {

    private Path intakeFirst, park;
    private PathChain launchFirst, prepareForIntakeSecond, intakeSecond1, backIntakeSecond, intakeSecond2, launchSecond;

    static Pose startingPose = new Pose(88.0, 7.5, Math.toRadians(90));
    static Pose controlPick1Point = new Pose(83, 37.0, Math.toRadians(0));
    static Pose pick1Pose = new Pose(130.0, 35.0, Math.toRadians(0));
    static Pose shoot1Pose = new Pose(87.0, 15.0, Math.toRadians(0));
    static Pose preparePick2Pose = new Pose(112.0, 7.0, Math.toRadians(0));
    static Pose pick2Pose = new Pose(135.0, 7.0, Math.toRadians(0));
    static Pose shoot2Pose = new Pose(87.0, 15.0, Math.toRadians(0));
    static Pose parkPose = new Pose(100.0, 25.0, Math.toRadians(0));

    Command autoCommand;

    public FarAuto_GLOBAL(Aliance aliance) {
        super(aliance, true);
    }

    public void createPaths() {

        if (currentAliance == Aliance.RED) {

            intakeFirst = new Path(new BezierCurve(startingPose, controlPick1Point, pick1Pose));
            intakeFirst.setTangentHeadingInterpolation();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick1Pose,
                            shoot1Pose))
                    .setLinearHeadingInterpolation(
                            pick1Pose.getHeading(),
                            shoot1Pose.getHeading()
                    )
                    .build();

            prepareForIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot1Pose,
                            preparePick2Pose))
                    .setLinearHeadingInterpolation(
                            launchFirst.endPose().getHeading(),
                            preparePick2Pose.getHeading())
                    .build();

            intakeSecond1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            preparePick2Pose,
                            pick2Pose))
                    .setConstantHeadingInterpolation(
                            pick2Pose.getHeading())
                    .build();

            backIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick2Pose,
                            preparePick2Pose))
                    .setConstantHeadingInterpolation(
                            pick2Pose.getHeading())
                    .build();

            intakeSecond2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            preparePick2Pose,
                            pick2Pose))
                    .setConstantHeadingInterpolation(
                            pick2Pose.getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick2Pose,
                            shoot2Pose))
                    .setConstantHeadingInterpolation(shoot2Pose.getHeading())
                    .build();

            park = new Path(new BezierLine(shoot2Pose, parkPose));
            park.setConstantHeadingInterpolation(parkPose.getHeading());

        } else {
            intakeFirst = new Path(new BezierCurve(
                    startingPose.mirror(),
                    controlPick1Point.mirror(),
                    pick1Pose.mirror()));
            intakeFirst.setTangentHeadingInterpolation();

            launchFirst = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick1Pose.mirror(),
                            shoot1Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            prepareForIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot1Pose.mirror(),
                            preparePick2Pose.mirror()))
                    .setLinearHeadingInterpolation(
                            launchFirst.endPose().getHeading(),
                            preparePick2Pose.mirror().getHeading())
                    .build();

            intakeSecond1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            preparePick2Pose.mirror(),
                            pick2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pick2Pose.mirror().getHeading())
                    .setTimeoutConstraint(1)
                    .build();

            backIntakeSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick2Pose.mirror(),
                            preparePick2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pick2Pose.mirror().getHeading())
                    .setTimeoutConstraint(1)
                    .build();

            intakeSecond2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            preparePick2Pose.mirror(),
                            pick2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pick2Pose.mirror().getHeading())
                    .build();

            launchSecond = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pick2Pose.mirror(),
                            shoot2Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            shoot2Pose.mirror().getHeading())
                    .build();

            park = new Path(new BezierLine(
                    shoot2Pose.mirror(),
                    parkPose.mirror()));
            park.setConstantHeadingInterpolation(
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

                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new WaitCommand(300)

        ).schedule();

        createPaths();

        autoCommand =
                new SequentialCommandGroup(

                        new ParallelDeadlineGroup(

                                new WaitCommand(3800),

                                new SequentialCommandGroup(
                                        new WaitCommand(2100),
                                        new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos)

                                ),

                                new ConditionalCommand(
                                        new turretToPosCMD(turretSb, 13.0),
                                        new turretToPosCMD(turretSb,-13.0),
                                        ()-> currentAliance.equals(Aliance.RED)),

                                new shooterToVelCMD(shooterSb, 1500)

                                /*new shooterToBasketCMD(shooterSb, visionSb, 1480)
                                */

                        ),

                        ///PRELOAD_LAUNCHED

                        stopShootCMD(false),

                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(intakeFirst).withTimeout(2300),

                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new moveIntakeAutonomousCMD(intakeSb, 1, 0.8)
                                )),

                        new WaitCommand(300),

                        new shooterToVelCMD(shooterSb, 1480),

                        new ConditionalCommand(
                        new turretToPosCMD(turretSb, -60.0),
                                new turretToPosCMD(turretSb,60.0),
                                ()-> currentAliance.equals(Aliance.RED)),

                                new moveIntakeAutonomousCMD(intakeSb, 0.4, 0),

                        pedroSb.followPathCmd(launchFirst).withTimeout(2300),


                        new WaitCommand(800),

                        shootThreeSpamerFarCMD(1500),

                        /// FIRST_LAUNCHED

                        stopShootCMD(false),

                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(prepareForIntakeSecond).withTimeout(2000),

                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new moveIntakeAutonomousCMD(intakeSb, 1, 1)
                                )),

                        pedroSb.followPathCmd(intakeSecond1).withTimeout(1000),

                        pedroSb.followPathCmd(backIntakeSecond).withTimeout(500),

                        pedroSb.followPathCmd(intakeSecond2).withTimeout(1000),

                        new WaitCommand(300),

                        new shooterToVelCMD(shooterSb, 1490),

                        new ConditionalCommand(
                                new turretToPosCMD(turretSb, -64.0),
                                new turretToPosCMD(turretSb,64.0),
                                ()-> currentAliance.equals(Aliance.RED)),
                        new moveIntakeAutonomousCMD(intakeSb, 0.4, 0),

                        pedroSb.followPathCmd(launchSecond).withTimeout(2300),

                        new WaitCommand(600),

                        shootThreeSpamerFarCMD(1500),

                        /// SECOND_LAUNCHED

                        stopShootCMD(false),

                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(prepareForIntakeSecond).withTimeout(2300),

                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new moveIntakeAutonomousCMD(intakeSb, 1, 0.8)
                                )),

                        new moveIntakeAutonomousCMD(intakeSb, 1, 0.7),

                        pedroSb.followPathCmd(intakeSecond1).withTimeout(1000),

                        pedroSb.followPathCmd(backIntakeSecond).withTimeout(500),

                        pedroSb.followPathCmd(intakeSecond2).withTimeout(1000),

                        new WaitCommand(300),

                        new shooterToVelCMD(shooterSb, 1490),

                        new ConditionalCommand(
                                new turretToPosCMD(turretSb, -64.0),
                                new turretToPosCMD(turretSb,64.0),
                                ()-> currentAliance.equals(Aliance.RED)
                        ),

                        new moveIntakeAutonomousCMD(intakeSb, 0.4, 0),

                        pedroSb.followPathCmd(launchSecond).withTimeout(2300),

                        new WaitCommand(700),

                        new WaitCommand(600),

                        shootThreeSpamerFarCMD(1500),
                        /// THIRD_LAUNCHED

                        stopShootCMD(false),






                        pedroSb.followPathCmd(park)

                        ///PARK


                );


    }

    @Override
    public void start() {
        autoCommand.schedule();

    }
}
