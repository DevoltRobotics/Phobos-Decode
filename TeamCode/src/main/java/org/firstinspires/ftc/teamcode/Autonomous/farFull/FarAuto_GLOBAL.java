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
    private PathChain intakeFirst, backIntakeFirst, launchFirst, prepareIntakeSeond, intakeSecond, launchSecond, pickCorner, pickCenter, launchCorner, launchCenter;


    Pose startingPose = m(new Pose(88.0, 8.2, Math.toRadians(0)));
    static Pose pick1Pose = new Pose(131.0, 11, Math.toRadians(354));
    static Pose backPick1Pose = new Pose(128.0, 10, Math.toRadians(0));
    static Pose front1Pose = new Pose(131.0, 11, Math.toRadians(354));

    static Pose shoot1Pose = new Pose(96, 9, Math.toRadians(25));
    static Pose preparePick2Pose = new Pose(100, 35.0, Math.toRadians(0));
    static Pose pick2Pose = new Pose(130.0, 35, Math.toRadians(0));
    static Pose shoot2Pose = new Pose(92.0, 10.0, Math.toRadians(0));

    static Pose pickCornerPose = new Pose(128.0, 9, Math.toRadians(0));

    static Pose pickCenterControlPoint = new Pose(93.0, 40.0, Math.toRadians(0));
    static Pose pickCenterPose = new Pose(130.0, 36, Math.toRadians(0));


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
                .setTangentHeadingInterpolation()
                .build();

        backIntakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick1Pose,
                        backPick1Pose))
                .addPath(new BezierLine(
                        backPick1Pose,
                        pick1Pose))
                .setConstantHeadingInterpolation(pick1Pose.getHeading())
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
                .setConstantHeadingInterpolation(shoot2Pose.getHeading())
                .build();

        pickCorner = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot2Pose,
                        pickCornerPose))
                .setTangentHeadingInterpolation()
                .build();

        pickCenter = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot2Pose,
                        pickCenterPose))
                .setTangentHeadingInterpolation()
                .build();

        launchCorner = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickCornerPose,
                        shoot2Pose))
                .setLinearHeadingInterpolation(
                        pickCornerPose.getHeading(),
                        shoot2Pose.getHeading()
                )
                .build();

        launchCenter = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickCenterPose,
                        shoot2Pose))
                .setLinearHeadingInterpolation(
                        pickCenterPose.getHeading(),
                        shoot2Pose.getHeading()
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
                        new ParallelRaceGroup(
                                new aimCMD(shooterSb, ()-> false),
                                new SequentialCommandGroup(
                                        new WaitCommand(3700),
                                        new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),

                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),

                                        new WaitCommand(1000)

                                )),

                        ///PRELOAD_LAUNCHED

                        stopShootCMD(false),

                        new InstantCommand(() -> intakeSb.setIntakePower(1, 0.8)),

                        pedroSb.followPathCmd(intakeFirst, 0.7).withTimeout(2300),
                        pedroSb.followPathCmd(backIntakeFirst).withTimeout(2300),

                        new WaitCommand(200),

                        new InstantCommand(() -> intakeSb.setIntakePower(0.5, 0)),

                        new WaitCommand(500),

                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(launchFirst),
                                        shootThreeSpamerFarCMD()
                                ),

                                new aimCMD(shooterSb, false, true)
                        ),
                        /// FIRST_LAUNCHED

                        stopShootCMD(false),

                        new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),

                        pedroSb.followPathCmd(intakeSecond).withTimeout(1500),

                        new WaitCommand(200),

                        new InstantCommand(() -> intakeSb.setIntakePower(0.5, 0)),


                        new WaitCommand(500),

                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        pedroSb.followPathCmd(launchSecond),
                                        shootThreeSpamerFarCMD()
                                ),

                                new aimCMD(shooterSb, false, true)
                        ),

                        /// SECOND_LAUNCHED

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

    }

}
