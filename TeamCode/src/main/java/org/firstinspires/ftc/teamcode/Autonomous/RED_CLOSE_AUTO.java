package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstArtifactsCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseRed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

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
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Autonomous
public class RED_CLOSE_AUTO extends OpModeCommand {

    private Path launchPreloadArtifacts, park;
    private PathChain prepareForIntakeArtifacts, intakeArtifacts, launchSecondArtifacts;

    Command autoCommand;

    public RED_CLOSE_AUTO() {
        super(Alliance.RED, true);
    }

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
                new SequentialCommandGroup(
                        new WaitCommand(13000),
                        new lateralBlockersCMD(sorterSb, 0, blockersUp),
                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                        new WaitCommand(500),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(launchPreloadArtifacts),
                                new turretToPosCMD(turretSb, 0),
                                new shooterToVelCMD(shooterSb,1120)
                        ),

                        new WaitCommand(1500),

                        new ParallelDeadlineGroup(

                                new WaitCommand(6000), // deadline

                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                                        new WaitCommand(1300),
                                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp),
                                        new WaitCommand(3500)

                                ),

                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new moveIntakeCMD(intakeSb, 1)
                                ),

                          //      new turretToBasketCMD(turretSb, visionSb, follower),
                                new shooterToBasketCMD(shooterSb, visionSb)

                        ),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(park),
                                new shooterToVelCMD(shooterSb,0),
                                new turretToPosCMD(turretSb, 0),

                                new SequentialCommandGroup(
                                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                                        new lateralBlockersCMD(sorterSb, 0, 0)

                                )
                        )
                );

    }

    @Override
    public void start() {
        autoCommand.schedule();

    }
}
