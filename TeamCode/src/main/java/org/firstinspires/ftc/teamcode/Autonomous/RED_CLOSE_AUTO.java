package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeSecondCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.intakeThirdCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchPreloadCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchSecondCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.launchThirdCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.openGateCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.openGateControlPointCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.parkCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeFirstCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeSecondCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.prepareForIntakeThirdCloseRedPose;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseRed;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Autonomous
public class RED_CLOSE_AUTO extends OpModeCommand {

    private Path launchPreload, park;
    private PathChain prepareForIntakeFirst, intakeFirst, openGate, launchFirst, prepareForIntakeSecond, intakeSecond, launchSecond, prepareForIntakeThird, intakeThird, launchThird;

    Command autoCommand;

    public RED_CLOSE_AUTO() {
        super(Alliance.RED, true);
    }

    public void createPaths() {
        launchPreload = new Path(new BezierLine(startingPoseCloseRed, launchPreloadCloseRedPose));
        launchPreload.setConstantHeadingInterpolation(startingPoseCloseRed.getHeading());

        prepareForIntakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(launchPreloadCloseRedPose, prepareForIntakeFirstCloseRedPose))
                .setConstantHeadingInterpolation(prepareForIntakeFirstCloseRedPose.getHeading())
                .build();

        intakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(prepareForIntakeFirstCloseRedPose, intakeFirstCloseRedPose))
                .setLinearHeadingInterpolation(prepareForIntakeFirstCloseRedPose.getHeading(), intakeFirstCloseRedPose.getHeading())

                .build();

        openGate= follower.pathBuilder()
                .addPath(new BezierCurve(intakeFirstCloseRedPose, openGateControlPointCloseRedPose, openGateCloseRedPose))
                .setLinearHeadingInterpolation(intakeFirstCloseRedPose.getHeading(), openGateCloseRedPose.getHeading())

                .build();

        launchFirst = follower.pathBuilder()
                .addPath(new BezierLine(openGateCloseRedPose, launchFirstCloseRedPose))
                .setLinearHeadingInterpolation(openGateCloseRedPose.getHeading(), launchFirstCloseRedPose.getHeading())

                .build();

        prepareForIntakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(launchFirstCloseRedPose, prepareForIntakeSecondCloseRedPose))
                .setLinearHeadingInterpolation(launchFirstCloseRedPose.getHeading(), prepareForIntakeSecondCloseRedPose.getHeading())
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(prepareForIntakeSecondCloseRedPose, intakeSecondCloseRedPose))
                .setLinearHeadingInterpolation(prepareForIntakeSecondCloseRedPose.getHeading(), intakeSecondCloseRedPose.getHeading())

                .build();

        launchSecond = follower.pathBuilder()
                .addPath(new BezierLine(intakeSecondCloseRedPose, launchSecondCloseRedPose))
                .setLinearHeadingInterpolation(intakeSecondCloseRedPose.getHeading(), launchSecondCloseRedPose.getHeading())

                .build();

        prepareForIntakeThird = follower.pathBuilder()
                .addPath(new BezierLine(launchSecondCloseRedPose, prepareForIntakeThirdCloseRedPose))
                .setLinearHeadingInterpolation(launchSecondCloseRedPose.getHeading(), prepareForIntakeThirdCloseRedPose.getHeading())
                .build();

        intakeThird = follower.pathBuilder()
                .addPath(new BezierLine(prepareForIntakeThirdCloseRedPose, intakeThirdCloseRedPose))
                .setLinearHeadingInterpolation(prepareForIntakeThirdCloseRedPose.getHeading(), intakeThirdCloseRedPose.getHeading())

                .build();

        launchThird = follower.pathBuilder()
                .addPath(new BezierLine(intakeThirdCloseRedPose, launchThirdCloseRedPose))
                .setLinearHeadingInterpolation(intakeThirdCloseRedPose.getHeading(), launchThirdCloseRedPose.getHeading())
                .build();


        park = new Path(new BezierLine(launchThirdCloseRedPose, parkCloseRedPose));
        park.setLinearHeadingInterpolation(launchThirdCloseRedPose.getHeading(), parkCloseRedPose.getHeading());

        //pickcenter = follower.holdPoint();


    }

    @Override
    public void initialize() {
        follower.setStartingPose(startingPoseCloseRed);

        createPaths();

        /*
        autoCommand =
                new SequentialCommandGroup(
                        new WaitCommand(13000),
                        new lateralBlockersCMD(sorterSb, 0, blockersUp),
                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                        new WaitCommand(500),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(launchPreload),
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

    }*/

        autoCommand =
                new SequentialCommandGroup(
                        pedroSb.followPathCmd(launchPreload),

                        pedroSb.followPathCmd(prepareForIntakeFirst),

                        new InstantCommand(
                                ()-> pedroSb.follower.setMaxPower(0.7)
                        ),
                        pedroSb.followPathCmd(intakeFirst),

                        new InstantCommand(
                                ()-> pedroSb.follower.setMaxPower(0.9)
                        ),
                        pedroSb.followPathCmd(openGate),

                        pedroSb.followPathCmd(launchFirst),

                        pedroSb.followPathCmd(prepareForIntakeSecond),

                        new InstantCommand(
                                ()-> pedroSb.follower.setMaxPower(0.65)
                        ),
                        pedroSb.followPathCmd(intakeSecond),

                        new InstantCommand(
                                ()-> pedroSb.follower.setMaxPower(0.9)
                        ),
                        pedroSb.followPathCmd(launchSecond),

                        pedroSb.followPathCmd(prepareForIntakeThird),
                        pedroSb.followPathCmd(intakeThird),
                        pedroSb.followPathCmd(launchThird)


                );


    }

        @Override
    public void start() {
        autoCommand.schedule();

    }
}
