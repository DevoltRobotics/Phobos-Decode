package org.firstinspires.ftc.teamcode.Autonomous.farFull;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
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
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
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

public class FarAuto_GLOBAL extends OpModeCommand {

    public FarAuto_GLOBAL(Alliance aliance) {
        super(aliance, true, false);
    }

    private ElapsedTime timer = new ElapsedTime();

    Pose m(Pose p) {
        return Alliance.BLUE.equals(currentAlliance) ? p.mirror() : p;
    }

    private Path park;
    private PathChain intakeFirst, backIntakeFirst, launchFirst, prepareIntakeSeond, intakeSecond, launchSecond, adjustAngle, pickCorner, pickCenter, launchCorner, launchCenter;


    Pose startingPose = m(new Pose(88.0, 8.2, Math.toRadians(0)));
    static Pose pick1Pose = new Pose(130.0, 11, Math.toRadians(354));
    static Pose backPick1Pose = new Pose(128.0, 10, Math.toRadians(0));
    static Pose front1Pose = new Pose(132.0, 10, Math.toRadians(0));

    static Pose shoot1Pose = new Pose(95, 14, Math.toRadians(0));
    static Pose preparePick2Pose = new Pose(100, 35.0, Math.toRadians(0));
    static Pose pick2Pose = new Pose(130.0, 35, Math.toRadians(0));
    static Pose shoot2Pose = new Pose(92.0, 14.0, Math.toRadians(25));

    static Pose pickCornerPose = new Pose(131.0, 9, Math.toRadians(0));

    static Pose pickCenterControlPoint = new Pose(93.0, 40.0, Math.toRadians(0));
    static Pose pickCenterPose = new Pose(132.0, 28, Math.toRadians(0));

    static Pose parkPose = new Pose(100.0, 25.0, Math.toRadians(0));
    Command autoCommand;

    public void createPaths() {

        intakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        startingPose,
                        pick1Pose))
                .addPath(new BezierLine(
                        backPick1Pose,
                        pick1Pose))
                .setLinearHeadingInterpolation(
                        backPick1Pose.getHeading(),
                        pick1Pose.getHeading()

                )
                .build();

        backIntakeFirst = follower.pathBuilder()
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

        launchFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick1Pose,
                        shoot1Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        prepareIntakeSeond = follower.pathBuilder()
                .addPath(new BezierLine(
                        preparePick2Pose,
                        pick2Pose))
                .setConstantHeadingInterpolation(
                        pick2Pose.getHeading())
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        preparePick2Pose,
                        pick2Pose))
                .setTangentHeadingInterpolation()
                .build();

        launchSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick2Pose,
                        shoot2Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


         adjustAngle = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot2Pose,
                        shoot1Pose))
                 .setLinearHeadingInterpolation(
                         shoot2Pose.getHeading(),
                         shoot1Pose.getHeading()
                 )
                .build();

        pickCorner = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot1Pose,
                        pickCornerPose))
                .setTangentHeadingInterpolation()
                .build();

        pickCenter = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot1Pose,
                        pickCenterPose))
                .setTangentHeadingInterpolation()
                .build();

        launchCorner = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickCornerPose,
                        shoot1Pose))
                .setLinearHeadingInterpolation(
                        pickCornerPose.getHeading(),
                        shoot1Pose.getHeading()
                )
                .build();

        launchCenter = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickCenterPose,
                        shoot1Pose))
                .setLinearHeadingInterpolation(
                        pickCenterPose.getHeading(),
                        shoot1Pose.getHeading()
                )
                .build();

        park = new Path(new BezierLine(shoot2Pose, parkPose));
        park.setConstantHeadingInterpolation(parkPose.getHeading());


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

                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new WaitCommand(300)

        ).schedule();

        createPaths();

        autoCommand =
                new SequentialCommandGroup(
                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new ParallelRaceGroup(
                                new aimCMD(shooterSb, false, () -> false),
                                new WaitCommand(3700)
                        ),

                        new ParallelRaceGroup(
                                new aimCMD(shooterSb, true, () -> false),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),

                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                                        new WaitCommand(850)

                                )),

                        ///PRELOAD_LAUNCHED

                        stopShootCMD(false),

                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                        pedroSb.followPathCmd(intakeFirst).withTimeout(2300),
                        pedroSb.followPathCmd(backIntakeFirst).withTimeout(2300),

                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(launchFirst),
                                        new WaitCommand(200)
                                ),

                                new aimCMD(shooterSb, false, () -> false)
                        ),

                        shootThreeSpamerFarCMD(),

                        /// FIRST_LAUNCHED

                        stopShootCMD(false),

                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                        pedroSb.followPathCmd(intakeSecond).withTimeout(1500),

                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(launchSecond),
                                        new WaitCommand(200)
                                ),

                                new aimCMD(shooterSb, false, () -> false)
                        ),

                        shootThreeSpamerFarCMD(),

                        /// SECOND_LAUNCHED

                        stopShootCMD(false),

                        pedroSb.followPathCmd(adjustAngle),

                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(pickCorner),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(launchCorner),
                                                        new WaitCommand(200)
                                                ),

                                                new aimCMD(shooterSb, false, () -> false)
                                        )),
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(pickCenter),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(launchCenter),
                                                        new WaitCommand(200)
                                                ),

                                                new aimCMD(shooterSb, false, () -> false)
                                        )), () -> visionSb.isArtifactsCorner()

                        ),

                        shootThreeSpamerFarCMD(),

                        stopShootCMD(false),

                        new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(pickCorner),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(launchCorner),
                                                        new WaitCommand(200)
                                                ),

                                                new aimCMD(shooterSb, false, () -> false)
                                        )),
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(pickCenter),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(launchCenter),
                                                        new WaitCommand(200)
                                                ),

                                                new aimCMD(shooterSb, false, () -> false)
                                        )), () -> visionSb.isArtifactsCorner()

                        ),

                        shootThreeSpamerFarCMD(),

                        stopShootCMD(false),

                        new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),


                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(pickCorner),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(launchCorner),
                                                        new WaitCommand(200)
                                                ),

                                                new aimCMD(shooterSb, false, () -> false)
                                        )),
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(pickCenter),
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        pedroSb.followPathCmd(launchCenter),
                                                        new WaitCommand(200)
                                                ),

                                                new aimCMD(shooterSb, false, () -> false)
                                        )), () -> visionSb.isArtifactsCorner()

                        ),

                        shootThreeSpamerFarCMD(),

                        stopShootCMD(false)

                        ///PARK
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
