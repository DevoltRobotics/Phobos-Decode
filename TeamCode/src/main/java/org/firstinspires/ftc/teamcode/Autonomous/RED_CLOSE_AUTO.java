package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeArtifactsCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeArtifactsControlPointCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstArtifactsCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondArtifactsCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeArtifactsCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeArtifactsControlPointCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseRed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.artifacToArtifactTimer;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.waitAimTimer;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.preSorterCmd;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.detectMotifCMD;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Autonomous
public class RED_CLOSE_AUTO extends OpModeCommand {

    private Path launchPreloadArtifacts, park;
    private PathChain prepareForIntakeArtifacts, intakeArtifacts, launchSecondArtifacts;


    Command autoCommand;

    public void createPaths() {
        launchPreloadArtifacts = new Path(new BezierLine(startingPoseCloseRed, launchFirstArtifactsCloseRedPose));
        launchPreloadArtifacts.setLinearHeadingInterpolation(startingPoseCloseRed.getHeading(), launchFirstArtifactsCloseRedPose.getHeading());

        /*prepareForIntakeArtifacts = follower.pathBuilder()
                .addPath(new BezierCurve(launchFirstArtifactsCloseRedPose, prepareForIntakeArtifactsControlPointCloseRedPose, prepareForIntakeArtifactsCloseRedPose))
                .setLinearHeadingInterpolation(launchFirstArtifactsCloseRedPose.getHeading(), prepareForIntakeArtifactsCloseRedPose.getHeading())
                .build();

        intakeArtifacts = follower.pathBuilder()
                .addPath(new BezierCurve(prepareForIntakeArtifactsCloseRedPose, intakeArtifactsControlPointCloseRedPose, intakeArtifactsCloseRedPose))
                .setLinearHeadingInterpolation(prepareForIntakeArtifactsCloseRedPose.getHeading(), intakeArtifactsCloseRedPose.getHeading())

                .build();

        launchSecondArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(intakeArtifactsCloseRedPose, launchSecondArtifactsCloseRedPose))
                .setLinearHeadingInterpolation(intakeArtifactsCloseRedPose.getHeading(), launchSecondArtifactsCloseRedPose.getHeading())

                .build();

         */

        park = new Path(new BezierLine(launchFirstArtifactsCloseRedPose, parkCloseRedPose));
        park.setLinearHeadingInterpolation(launchFirstArtifactsCloseRedPose.getHeading(), parkCloseRedPose.getHeading());

        //pickcenter = follower.holdPoint();


    }

    @Override
    public void initialize() {
        follower.setStartingPose(startingPoseCloseRed);

        createPaths();

        autoCommand =
                new ParallelCommandGroup(
                        pedroSb.followPathCmd(launchPreloadArtifacts),
                        new turretToPosCMD(turretSb, -70)


                ).andThen(
                        new SequentialCommandGroup(
                                new detectMotifCMD(visionSb),

                                new ParallelCommandGroup(
                                new ParallelDeadlineGroup(
                                        new WaitCommand(1200),
                                        new turretToPosCMD(turretSb, 0),
                                        new shooterToVelAutonomousCMD(shooterSb,1000)

                                ),  new ParallelDeadlineGroup(
                                new preSorterCmd(sorterSb, visionSb.pattern, sorterSb.rightArtifact, sorterSb.leftArtifact),
                                new moveIntakeCMD(intakeSb, 1)
                        )
                        )

                        )

                ).andThen(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(waitAimTimer),
                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                                        new WaitCommand(artifacToArtifactTimer),
                                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                                        new WaitCommand(2500)

                                        ),

                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(waitAimTimer - 50),
                                                new moveIntakeCMD(intakeSb, 1)
                                                ),
                                    new turretToBasketCMD(turretSb, visionSb),
                                    new shooterToBasketCMD(shooterSb, visionSb)
                                )
                        )

                ).andThen(
                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(park),
                                new shooterToVelAutonomousCMD(shooterSb,0),

                                new SequentialCommandGroup(
                                    new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                                        new lateralBlockersCMD(sorterSb, 0, 0)

                                )
                                )
                );

                        /*.andThen(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                pedroSb.followPathCmd(intakeArtifacts),
                                new WaitCommand(800)
                                ),

                                new moveIntakeCMD(intakeSb, 1)
                                )

                ).andThen(
                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(launchSecondArtifacts),
                                new turretToPosCMD(turretSb, -75),
                                new shooterToVelAutonomousCMD(shooterSb,1000),

                        new ParallelDeadlineGroup(
                                        new preSorterCmd(sorterSb, visionSb.pattern, sorterSb.rightArtifact, sorterSb.leftArtifact),
                                        new moveIntakeCMD(intakeSb, 1)
                                )
                        )
                ).andThen(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(waitAimTimer),
                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                                        new WaitCommand(artifacToArtifactTimer),
                                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                                        new WaitCommand(2000)

                                ),

                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(waitAimTimer - 50),
                                                new moveIntakeCMD(intakeSb, 1)
                                        ),
                                        new turretToBasketCMD(turretSb, visionSb),
                                        new shooterToBasketCMD(shooterSb, visionSb)
                                )
                        )
                ).andThen(
                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(park),
                                new shooterToVelAutonomousCMD(shooterSb, 0),
                                new turretToPosCMD(turretSb, 0)
                        )

                         */

    }

    @Override
    public void init() {
        new lateralBlockersCMD(sorterSb, 0, 0).schedule();
        new horizontalBlockerCMD(sorterSb, blockerHHidePos).schedule();

    }

    @Override
    public void start() {
        autoCommand.schedule();

    }
}
