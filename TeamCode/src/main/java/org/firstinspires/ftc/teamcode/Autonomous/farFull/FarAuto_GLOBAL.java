package org.firstinspires.ftc.teamcode.Autonomous.farFull;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.downRampPos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.aimCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.setIsCorner;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;

public class FarAuto_GLOBAL extends OpModeCommand {

    public FarAuto_GLOBAL(Alliance aliance) {
        super(aliance, true, false);
    }

    private ElapsedTime timer = new ElapsedTime();

    Command autoCommand, parkCommand;

    private Path park;
    private PathChain shootPreload, intakeFirst, backIntakeFirst, launchFirst, prepareIntakeSeond, intakeSecond, launchSecond, adjustAngle, pickCorner, pickCenter, launchCorner, launchCenter;


    Pose m(Pose p) {
        return Alliance.BLUE.equals(currentAlliance) ? p.mirror() : p;
    }
    Pose startingPose = m(new Pose(88.0, 8.2, Math.toRadians(0)));
    Pose shootPreloadPose = m(new Pose(84, 21, Math.toRadians(20)));

    Pose pick2Pose = m(new Pose(130.0, 37, Math.toRadians(20)));
    Pose shoot2Pose = m(new Pose(90.0, 15.0, Math.toRadians(0)));

    Pose pick1Pose = m(new Pose(132.0, 10, Math.toRadians(0)));
    Pose backPick1Pose = m(new Pose(128.0, 10, Math.toRadians(0)));
    Pose front1Pose = m(new Pose(132.0, 10, Math.toRadians(0)));

    Pose shoot1Pose = m(new Pose(91, 14, Math.toRadians(0)));
    Pose preparePick2Pose = m(new Pose(103.0, 35, Math.toRadians(15)));

    Pose pick2ControlPoint = m(new Pose(93, 40, Math.toRadians(0)));

    Pose shootCyclesPose = m(new Pose(91, 14, Math.toRadians(5)));

    Pose pickCornerPose = m(new Pose(133.0, 10, Math.toRadians(0)));

    Pose pickCenterControlPoint = m(new Pose(93.0, 40.0, Math.toRadians(0)));
    Pose pickCenterPose = m(new Pose(132.0, 27, Math.toRadians(0)));

    Pose parkPose = m(new Pose(105, 14.0, Math.toRadians(0)));

    public void createPaths() {

        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        startingPose,
                        shootPreloadPose))
                .setLinearHeadingInterpolation(
                        startingPose.getHeading(),
                        shootPreloadPose.getHeading())
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPreloadPose,
                        pick2Pose))
                .setTangentHeadingInterpolation()
                .build();

        launchSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick2Pose,
                        shoot2Pose))
                .setLinearHeadingInterpolation(
                        intakeSecond.endPose().getHeading(),
                        Alliance.BLUE.equals(currentAlliance) ? Math.toRadians(180) : 0

                )
                .setTimeoutConstraint(1)
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot2Pose,
                        pick1Pose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.8))
                .build();

        /*backIntakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick1Pose,
                        backPick1Pose))
                .setLinearHeadingInterpolation(
                        pick1Pose.getHeading(),
                        backPick1Pose.getHeading()
                )
                .addPath(new BezierLine(
                        backPick1Pose,
                        front1Pose))
                .setConstantHeadingInterpolation(front1Pose.getHeading())
                .build();

         */

        launchFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick1Pose,
                        shoot1Pose))
                .setLinearHeadingInterpolation(
                        pick1Pose.getHeading(),
                        shoot1Pose.getHeading()
                )
                .setTimeoutConstraint(1)
                .build();

        /*prepareIntakeSeond = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot1Pose,
                        preparePick2Pose))
                .setConstantHeadingInterpolation(
                        pick2Pose.getHeading())
                .setTimeoutConstraint(1)
                .build();

         */


        pickCorner = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootCyclesPose,
                        pickCornerPose))
                .setConstantHeadingInterpolation(Alliance.BLUE.equals(currentAlliance) ? Math.toRadians(180) : 0)
                .build();

        pickCenter = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootCyclesPose,
                        pickCenterPose))
                .setTangentHeadingInterpolation()
                .build();

        launchCorner = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickCornerPose,
                        shootCyclesPose))
                .setConstantHeadingInterpolation(Alliance.BLUE.equals(currentAlliance) ? Math.toRadians(175) : Math.toRadians(5))
                .setTimeoutConstraint(1)
                .build();

        launchCenter = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickCenterPose,
                        shoot1Pose))
                .setConstantHeadingInterpolation(Alliance.BLUE.equals(currentAlliance) ? Math.toRadians(175) : Math.toRadians(5))
                .setTimeoutConstraint(1)
                .build();

        park = new Path(new BezierLine(shoot1Pose, parkPose));
        park.setConstantHeadingInterpolation(parkPose.getHeading());

    }

    @Override
    public void initialize() {
        /*if (currentAlliance.equals(Alliance.RED)) {

        } else {
            follower.setStartingPose(startingPose.mirror());

        }

         */

        follower.setStartingPose(startingPose);

        new SequentialCommandGroup(
                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                new rampCMD(sorterSb, downRampPos),

                new WaitCommand(200),

                new rampCMD(sorterSb, upRampPos),

                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new WaitCommand(300)

        ).schedule();

        createPaths();

        parkCommand = new SequentialCommandGroup(
                stopShootCMD(false),
                new InstantCommand(() -> shooterSb.setShooterTarget(0)),
                new InstantCommand(() -> follower.setMaxPower(1)),

                pedroSb.followPathCmd(park)
        );

        autoCommand =
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> follower.setMaxPower(1)),

                                        new ParallelRaceGroup(
                                                new aimCMD(shooterSb, false, false, 30),
                                                new WaitCommand(3600),
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(shootPreload),
                                                        new WaitCommand(1000000)
                                                )
                                        ),

                                        new ParallelRaceGroup(
                                                new aimCMD(shooterSb, true, false),
                                                new SequentialCommandGroup(
                                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),

                                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                                                        new WaitCommand(880)

                                                )),

                                        ///PRELOAD_LAUNCHED

                                        stopShootCMD(false),

                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                        pedroSb.followPathCmd(intakeSecond).withTimeout(2500),

                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(launchSecond)
                                                ),

                                                new aimCMD(shooterSb, true, false)
                                        ),

                                        shootThreeSpamerFarCMD(),

                                        /// FIRST_LAUNCHED

                                        stopShootCMD(false),

                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                        pedroSb.followPathCmd(intakeFirst).withTimeout(1600),

                                        new InstantCommand(() -> follower.setMaxPower(1)),

                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(launchFirst)
                                                ),

                                                new aimCMD(shooterSb, true, false)
                                        ),

                                        new ParallelRaceGroup(
                                            shootThreeSpamerFarCMD(),
                                            new setIsCorner(visionSb)
                                        ),

                                        /// SECOND_LAUNCHED

                                        stopShootCMD(false),

                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(pickCorner).withTimeout(1500),

                                                        new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                        pedroSb.followPathCmd(launchCorner)
                                                                ),

                                                                new aimCMD(shooterSb, true, false)
                                                        )),
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(pickCenter).withTimeout(2200),

                                                        new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                        pedroSb.followPathCmd(launchCenter)
                                                                ),

                                                                new aimCMD(shooterSb, true, false)
                                                        )), () ->  visionSb.currentArtifactCorner

                                        ),

                                        new ParallelRaceGroup(
                                                shootThreeSpamerFarCMD(),
                                                new setIsCorner(visionSb)
                                        ),
                                        /// THIRD_LAUNCHED

                                        stopShootCMD(false),

                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(pickCorner).withTimeout(1500),
                                                        new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                        pedroSb.followPathCmd(launchCorner)
                                                                ),

                                                                new aimCMD(shooterSb, true, false)
                                                        )),
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(pickCenter).withTimeout(2200),

                                                        new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                        pedroSb.followPathCmd(launchCenter)
                                                                ),

                                                                new aimCMD(shooterSb, true, false)
                                                        )), () ->  visionSb.currentArtifactCorner

                                        ),

                                        new ParallelRaceGroup(
                                                shootThreeSpamerFarCMD(),
                                                new setIsCorner(visionSb)
                                        ),
                                        /// FOURTH_LAUNCHED

                                        stopShootCMD(false),

                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(pickCorner).withTimeout(1500),

                                                        new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                        pedroSb.followPathCmd(launchCorner)
                                                                ),

                                                                new aimCMD(shooterSb, true, false)
                                                        )),
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(pickCenter).withTimeout(2200),

                                                        new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                        pedroSb.followPathCmd(launchCenter)
                                                                ),

                                                                new aimCMD(shooterSb, true, false)
                                                        )), () ->  visionSb.currentArtifactCorner

                                        ),

                                        new ParallelRaceGroup(
                                                shootThreeSpamerFarCMD(),
                                                new setIsCorner(visionSb)
                                        ),
                                        /// FIVE_LAUNCHED
                                        stopShootCMD(false),

                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(pickCorner).withTimeout(1500),

                                                        new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                        pedroSb.followPathCmd(launchCorner)
                                                                ),

                                                                new aimCMD(shooterSb, true, false)
                                                        )),
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(pickCenter).withTimeout(2200),

                                                        new ParallelRaceGroup(
                                                                new SequentialCommandGroup(
                                                                        pedroSb.followPathCmd(launchCenter)
                                                                ),

                                                                new aimCMD(shooterSb, true, false)
                                                        )), () -> visionSb.currentArtifactCorner

                                        ),

                                        new ParallelRaceGroup(
                                                shootThreeSpamerFarCMD(),
                                                new setIsCorner(visionSb)
                                        )
                                        /// SIX_LAUNCHED

                                ),

                                new WaitCommand(29500)
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
