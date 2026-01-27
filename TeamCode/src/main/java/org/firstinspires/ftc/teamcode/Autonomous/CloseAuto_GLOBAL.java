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
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

public class CloseAuto_GLOBAL extends OpModeCommand {

    private Path launchPreload, park;
    private PathChain intakeFirst, launchFirst, openGate1, intakeSecond, launchSecond, openGate2, intakeThird, launchThird, intakeFive, launchFive, intakeFourth, launchFourth;

    private Pose currentStartingPose;
    Command autoCommand;

    static Pose startingPose = new Pose(123.0, 123.0, Math.toRadians(38));

    static Pose shootPreloadPose = new Pose(88.0, 84.0, Math.toRadians(38));

    static Pose pick1ControlPoint = new Pose(92, 60, Math.toRadians(0));

    static Pose pick1Pose = new Pose(134.0, 58.0, Math.toRadians(0));

    static Pose shoot1ControlPoint = new Pose(91.0, 66.0, Math.toRadians(0));
    static Pose shoot1Pose = new Pose(87.0, 77.0, Math.toRadians(330));

    static Pose opengate1ControlPoint = new Pose(109, 62.0, Math.toRadians(0));
    static Pose opengate1Pose = new Pose(128, 65.0, Math.toRadians(0));

    static Pose pick2Pose = new Pose(134.0, 57.0, Math.toRadians(36));

    static Pose shoot2ControlPoint = new Pose(99, 62.0, Math.toRadians(0));

    static Pose shoot2Pose = new Pose(86.0, 76.0, Math.toRadians(0));

    static Pose opengate2ControlPoint = new Pose(109, 62.0, Math.toRadians(0));
    static Pose opengate2Pose = new Pose(128, 65.0, Math.toRadians(0));

    static Pose pick3Pose = new Pose(134.0, 57.0, Math.toRadians(33));

    static Pose shoot3ControlPoint = new Pose(109, 62.0, Math.toRadians(0));

    static Pose shoot3Pose = new Pose(88.0, 84.0, Math.toRadians(0));

    static Pose pickUp4Pose = new Pose(127.0, 84.0, Math.toRadians(0));

    static Pose shoot4Pose = new Pose(88.0, 84.0, Math.toRadians(0));

    static Pose pickUp5ControlPoint = new Pose(88.0, 30.0, Math.toRadians(0));

    static Pose pickUp5Pose = new Pose(135.0, 35.0, Math.toRadians(0));

    static Pose shoot5ControlPoint = new Pose(115, 63, Math.toRadians(340));

    static Pose shoot5Pose = new Pose(87.0, 77.0, Math.toRadians(330));
    static Pose parkPose = new Pose(108.0, 70.0, Math.toRadians(0));

    public CloseAuto_GLOBAL(Aliance aliance) {
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
                    .addPath(new BezierLine(
                            opengate1Pose,
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


            openGate2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot2Pose,
                            opengate2ControlPoint,
                            opengate2Pose))
                    .setTangentHeadingInterpolation()
                    .setTimeoutConstraint(1)
                    .build();


            intakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(
                            opengate2Pose,
                            pick3Pose))
                    .setLinearHeadingInterpolation(
                            opengate2Pose.getHeading(),
                            pick3Pose.getHeading())
                    .build();

            launchThird = follower.pathBuilder()

                    .addPath(new BezierCurve(
                            pick3Pose,
                            shoot3ControlPoint,
                            shoot3Pose))
                    .setLinearHeadingInterpolation(
                            pick3Pose.getHeading(),
                            shoot3Pose.getHeading()
                    )
                    .setTimeoutConstraint(1)
                    .build();

            intakeFourth = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot3Pose,
                            pickUp4Pose))
                    .setConstantHeadingInterpolation(
                            pickUp4Pose.getHeading()
                    )
                    .build();

            launchFourth = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp4Pose,
                            shoot4Pose))
                    .setConstantHeadingInterpolation(
                            pickUp4Pose.getHeading()
                    )
                    .setTimeoutConstraint(1)
                    .build();

            intakeFive = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot4Pose,
                            pickUp5ControlPoint,
                            pickUp5Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            launchFive = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pickUp5Pose,
                            shoot5ControlPoint,
                            shoot5Pose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .setTimeoutConstraint(1)
                    .build();

            park = new Path(new BezierLine(shoot5Pose, parkPose));
            park.setLinearHeadingInterpolation(shoot5Pose.getHeading(), parkPose.getHeading());

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


            openGate2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot2Pose.mirror(),
                            opengate2ControlPoint.mirror(),
                            opengate2Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .setTimeoutConstraint(1)
                    .build();


            intakeThird = follower.pathBuilder()
                    .addPath(new BezierLine(
                            opengate2Pose.mirror(),
                            pick3Pose.mirror()))
                    .setLinearHeadingInterpolation(
                            opengate2Pose.mirror().getHeading(),
                            pick3Pose.mirror().getHeading())
                    .build();


            launchThird = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pick3Pose.mirror(),
                            shoot3ControlPoint.mirror(),
                            shoot3Pose.mirror()))
                    .setLinearHeadingInterpolation(
                            pick3Pose.mirror().getHeading(),
                            shoot3Pose.mirror().getHeading())
                    .setTimeoutConstraint(1)
                    .build();


            intakeFourth = follower.pathBuilder()
                    .addPath(new BezierLine(
                            shoot3Pose.mirror(),
                            pickUp4Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pickUp4Pose.mirror().getHeading())
                    .build();


            launchFourth = follower.pathBuilder()
                    .addPath(new BezierLine(
                            pickUp4Pose.mirror(),
                            shoot4Pose.mirror()))
                    .setConstantHeadingInterpolation(
                            pickUp4Pose.mirror().getHeading())
                    .setTimeoutConstraint(1)
                    .build();


            intakeFive = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            shoot4Pose.mirror(),
                            pickUp5ControlPoint.mirror(),
                            pickUp5Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();


            launchFive = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pickUp5Pose.mirror(),
                            shoot5ControlPoint.mirror(),
                            shoot5Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .setTimeoutConstraint(1)
                    .build();


            park = new Path(new BezierLine(
                    shoot5Pose.mirror(),
                    parkPose.mirror()));
            park.setLinearHeadingInterpolation(
                    shoot5Pose.mirror().getHeading(),
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

                new lateralBlockersCMD(sorterSb, blockersUp, 0), new horizontalBlockerCMD(sorterSb, blockerHHidePos)

        ).schedule();

        createPaths();
        autoCommand = new SequentialCommandGroup(

                new shooterToVelCMD(shooterSb, 1180),

                new moveIntakeAutonomousCMD(intakeSb, 0.2, 0),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -5.0),
                        new turretToPosCMD(turretSb, 5.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                new ParallelDeadlineGroup(

                        pedroSb.followPathCmd(launchPreload).withTimeout(2300),

                        new SequentialCommandGroup(
                                new WaitCommand(1050),
                                new moveIntakeAutonomousCMD(intakeSb, 1),

                                new WaitCommand(100),
                                new horizontalBlockerCMD(sorterSb, blockerHFreePos)
                        ),

                        new turretToBasketCMD(turretSb, visionSb)
                ),


                new WaitCommand(1050),
                ///PRELOAD_LAUNCHED

                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                pedroSb.followPathCmd(intakeFirst).withTimeout(2000),

                new shooterToVelCMD(shooterSb, 1250),

                new moveIntakeAutonomousCMD(intakeSb, 0.5, 0.3),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -70.0),
                        new turretToPosCMD(turretSb, 70.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                pedroSb.followPathCmd(launchFirst).withTimeout(2300),

                shootThreeSpamerCloseCMD(),

                /// FIRST_LAUNCHED

                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                pedroSb.followPathCmd(openGate1).withTimeout(1500),

                pedroSb.followPathCmd(intakeSecond).withTimeout(500),

                new shooterToVelCMD(shooterSb, 1250),

                new WaitCommand(800),

                new moveIntakeAutonomousCMD(intakeSb, -0.1, 0.8),

                new WaitCommand(100),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -42.0),
                        new turretToPosCMD(turretSb, 42.0),
                        () -> currentAliance.equals(Aliance.RED)),

                pedroSb.followPathCmd(launchSecond).withTimeout(2300),

                shootThreeSpamerCloseCMD(),

                /// SECOND_LAUNCHED

                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                pedroSb.followPathCmd(openGate2).withTimeout(1500),

                pedroSb.followPathCmd(intakeThird).withTimeout(500),

                new shooterToVelCMD(shooterSb, 1250),

                new WaitCommand(800),

                new moveIntakeAutonomousCMD(intakeSb, -0.1, 0.8),

                new WaitCommand(100),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -38.0),
                        new turretToPosCMD(turretSb, 38.0),
                        () -> currentAliance.equals(Aliance.RED)),

                        pedroSb.followPathCmd(launchThird).withTimeout(2300),


                shootThreeSpamerCloseCMD(),

                /// THIRD_LAUNCHED

                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                pedroSb.followPathCmd(intakeFourth).withTimeout(1000),

                new shooterToVelCMD(shooterSb, 1250),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -40.0),
                        new turretToPosCMD(turretSb, 40.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                pedroSb.followPathCmd(launchFourth).withTimeout(2300),

                shootThreeSpamerCloseCMD(),

                /// FOURTH_LAUNCHED


                stopShootCMD(false),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.8),

                pedroSb.followPathCmd(intakeFive).withTimeout(3000),

                new shooterToVelCMD(shooterSb, 1250),

                new WaitCommand(200),

                new moveIntakeAutonomousCMD(intakeSb, 0.5, 0.3),

                new ConditionalCommand(
                        new turretToPosCMD(turretSb, -30.0),
                        new turretToPosCMD(turretSb, 30.0),
                        () -> currentAliance.equals(Aliance.RED)
                ),

                pedroSb.followPathCmd(launchFive).withTimeout(2300),

                shootThreeSpamerCloseCMD(),

                /// FIVE_LAUNCHED

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

