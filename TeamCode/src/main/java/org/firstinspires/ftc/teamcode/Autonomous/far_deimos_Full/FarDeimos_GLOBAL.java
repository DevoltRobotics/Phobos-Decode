package org.firstinspires.ftc.teamcode.Autonomous.far_deimos_Full;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;

public class FarDeimos_GLOBAL extends OpModeCommand {

    private Path park;

    static Pose startingPose = new Pose(88.0, 7.5, Math.toRadians(0));
    static Pose parkPose = new Pose(100.0, 15.0, Math.toRadians(0));

    Command autoCommand;

    public FarDeimos_GLOBAL(Aliance aliance) {
        super(aliance, true);
    }

    public void createPaths() {

        if (currentAliance == Aliance.RED) {

            park = new Path(new BezierLine(startingPose, parkPose));
            park.setConstantHeadingInterpolation(parkPose.getHeading());

        } else {

            park = new Path(new BezierLine(startingPose.mirror(), parkPose.mirror()));
            park.setConstantHeadingInterpolation(parkPose.mirror().getHeading());
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
                        new WaitCommand(10),

                        new shooterToVelCMD(shooterSb, 1490),

                        new ConditionalCommand(
                                new turretToPosCMD(turretSb, -75.0),
                                new turretToPosCMD(turretSb, 75.0),
                                () -> currentAliance.equals(Aliance.RED)),

                        new WaitCommand(2300),

                        new ParallelDeadlineGroup(

                                new WaitCommand(2000),

                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new moveIntakeAutonomousCMD(intakeSb, 0.9, 0.7),

                                        new horizontalBlockerCMD(sorterSb, blockerHFreePos)

                                ),

                                new turretToBasketCMD(turretSb, visionSb)
                        ),

                        ///PRELOAD_LAUNCHED

                        stopShootCMD(false),

                        pedroSb.followPathCmd(park).withTimeout(1000),

                        new WaitCommand(600)

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
