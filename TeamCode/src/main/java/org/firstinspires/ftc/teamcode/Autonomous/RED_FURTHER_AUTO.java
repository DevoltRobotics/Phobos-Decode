package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeArtifactsFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondArtifactsFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeArtifactsFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseFurtherRed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.artifacToArtifactTimer;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.waitAimTimer;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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

public class RED_FURTHER_AUTO extends OpModeCommand {

    private Path prepareForIntakeArtifacts, park;
    private PathChain intakeArtifacts, launchSecondArtifacts;


    Command autoCommand;

    public void createPaths() {
        prepareForIntakeArtifacts = new Path(new BezierLine(startingPoseFurtherRed, prepareForIntakeArtifactsFurtherRedPose));
        prepareForIntakeArtifacts.setLinearHeadingInterpolation(startingPoseFurtherRed.getHeading(), prepareForIntakeArtifactsFurtherRedPose.getHeading());


        intakeArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(prepareForIntakeArtifactsFurtherRedPose, intakeArtifactsFurtherRedPose))
                .setLinearHeadingInterpolation(prepareForIntakeArtifactsFurtherRedPose.getHeading(), intakeArtifactsFurtherRedPose.getHeading())

                .build();

        launchSecondArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(intakeArtifactsFurtherRedPose, launchSecondArtifactsFurtherRedPose))
                .setLinearHeadingInterpolation(intakeArtifactsFurtherRedPose.getHeading(), launchSecondArtifactsFurtherRedPose.getHeading())

                .build();


        park = new Path(new BezierLine(launchSecondArtifactsFurtherRedPose, parkFurtherRedPose));
        park.setLinearHeadingInterpolation(launchSecondArtifactsFurtherRedPose.getHeading(), parkFurtherRedPose.getHeading());

        //pickcenter = follower.holdPoint();


    }

    @Override
    public void initialize() {
        follower.setStartingPose(startingPoseFurtherRed);

        createPaths();

        new lateralBlockersCMD(sorterSb, 0, 0).schedule();
        new horizontalBlockerCMD(sorterSb, blockerHHidePos).schedule();

        autoCommand =
                new ParallelCommandGroup(
                        new turretToPosCMD(turretSb, 25),
                        new shooterToVelAutonomousCMD(shooterSb,1420),

                        new ParallelDeadlineGroup(
                                new preSorterCmd(sorterSb, visionSb.pattern, sorterSb.rightArtifact, sorterSb.leftArtifact),
                                new moveIntakeCMD(intakeSb, 1)
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
                                pedroSb.followPathCmd(prepareForIntakeArtifacts),
                                new shooterToVelAutonomousCMD(shooterSb,0),


                                new SequentialCommandGroup(
                                new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                                new lateralBlockersCMD(sorterSb, blockersUp, blockersUp)

                                )
                                )

                ).andThen(
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
                                new turretToPosCMD(turretSb, 0),
                                new shooterToVelAutonomousCMD(shooterSb,1420),

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
                        new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(park),
                                new shooterToVelAutonomousCMD(shooterSb,0),
                                new turretToPosCMD(turretSb, 0)
                        )

                        );
    }

    @Override
    public void start() {
        autoCommand.schedule();
    }

    @Override
    public void init_loop() {
        new detectMotifCMD(visionSb).schedule();
    }
}
