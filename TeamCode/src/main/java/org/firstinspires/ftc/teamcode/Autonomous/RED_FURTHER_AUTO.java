package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkFurtherRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseFurtherRed;
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
public class RED_FURTHER_AUTO extends OpModeCommand {

    private Path prepareForIntakeArtifacts, park;
    private PathChain intakeArtifacts, launchSecondArtifacts;


    Command autoCommand;

    public RED_FURTHER_AUTO() {
        super(Alliance.BLUE, true);
    }

    public void createPaths() {
        /*prepareForIntakeArtifacts = new Path(new BezierLine(startingPoseFurtherRed, prepareForIntakeArtifactsFurtherRedPose));
        prepareForIntakeArtifacts.setLinearHeadingInterpolation(startingPoseFurtherRed.getHeading(), prepareForIntakeArtifactsFurtherRedPose.getHeading());


        intakeArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(prepareForIntakeArtifactsFurtherRedPose, intakeArtifactsFurtherRedPose))
                .setLinearHeadingInterpolation(prepareForIntakeArtifactsFurtherRedPose.getHeading(), intakeArtifactsFurtherRedPose.getHeading())

                .build();

        launchSecondArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(intakeArtifactsFurtherRedPose, launchSecondArtifactsFurtherRedPose))
                .setLinearHeadingInterpolation(intakeArtifactsFurtherRedPose.getHeading(), launchSecondArtifactsFurtherRedPose.getHeading())

                .build();

         */


        park = new Path(new BezierLine(startingPoseFurtherRed, parkFurtherRedPose));
        park.setConstantHeadingInterpolation(startingPoseFurtherRed.getHeading());

        //pickcenter = follower.holdPoint();


    }

    @Override
    public void initialize() {
        follower.setStartingPose(startingPoseFurtherRed);

        createPaths();

        autoCommand =
                new SequentialCommandGroup(

                        ///START

                        new lateralBlockersCMD(sorterSb, 0, blockersUp),
                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                        new turretToPosCMD(turretSb, 10),
                        new WaitCommand(900),

                        ///SORTER Y PREPARE SHOOTER/TURRET

                        new turretToPosCMD(turretSb, 20),
                        new shooterToVelCMD(shooterSb, 1460),

                        new WaitCommand(1500),

                        ///LAUNCH TO GOAL

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

                           //     new turretToBasketCMD(turretSb, visionSb, follower),
                                new shooterToBasketCMD(shooterSb, visionSb)
                        ),

                        ///PARK

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(park),
                                new shooterToVelCMD(shooterSb, 0),
                                new turretToPosCMD(turretSb, 0),

                                new SequentialCommandGroup(
                                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                                        new lateralBlockersCMD(sorterSb, 0, 0)

                                )
                        )
                );

                        /*

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

                         */
    }

    @Override
    public void start() {
        autoCommand.schedule();
    }

    @Override
    public void init_loop() {
    }
}
