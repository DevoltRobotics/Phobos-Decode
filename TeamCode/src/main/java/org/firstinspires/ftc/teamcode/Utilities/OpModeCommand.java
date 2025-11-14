package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.waitAimTimer;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.artifacToArtifactTimer;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Autonomous.DrivePos.startingPoseCloseRed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Lifting.LiftingSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.detectMotifCMD;

import java.util.function.BooleanSupplier;

@Config
public abstract class OpModeCommand extends OpMode {

    public Follower follower;

    public Alliance currentAliance = Alliance.RED;

    public TelemetryManager telemetryM;

    public VisionSubsystem visionSb;
    public PedroSubsystem pedroSb;
    public IntakeSubsystem intakeSb;
    public SorterSubsystem sorterSb;
    public LiftingSubsystem liftingSubsystem;
    public ShooterSubsystem shooterSb;
    public TurretSubsystem turretSb;

    public IMU imu;

    //reinicia la lista de comandos
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    //corre el scheduler
    public void run() {
        CommandScheduler.getInstance().run();
    }

    //programa comandos al scheduler
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    //registra subsistemas al scheduler
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPoseCloseRed == null ? new Pose() : startingPoseCloseRed);
        follower.update();


        register(
                pedroSb = new PedroSubsystem(follower, telemetry),
                intakeSb = new IntakeSubsystem(hardwareMap),
                sorterSb = new SorterSubsystem(hardwareMap, telemetry),
                turretSb = new TurretSubsystem(hardwareMap, telemetry, true),
                //liftingSubsystem = new LiftingSubsystem(hardwareMap),
                shooterSb = new ShooterSubsystem(hardwareMap, telemetry, true),
                visionSb = new VisionSubsystem(hardwareMap, currentAliance)
        );

        visionSb.limelight.pipelineSwitch(0);

        //imu = hardwareMap.get(IMU.class, "imu");

        initialize();
    }

    /*public void initImu() {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
    }

     */

    @Override
    public void init_loop() {
        telemetry.addData("PATTERN", visionSb.alliance);

        new detectMotifCMD(visionSb).schedule();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        visionSb.limelight.pipelineSwitch(0);

    }

    @Override
    public void loop() {
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));

        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
        run();
    }

    public void stop() {
        reset();
    }

    public abstract void initialize();


    //COMANDS

    public Command cyclesShootCMD(BooleanSupplier booleano) {

        return new ParallelCommandGroup(
                new turretToBasketCMD(turretSb, visionSb).asProxy().interruptOn(booleano),
                new shooterToBasketCMD(shooterSb, visionSb).asProxy(),

                new SequentialCommandGroup(
                        new WaitCommand(waitAimTimer - 50),
                        new moveIntakeCMD(intakeSb, 1)
                ),

                new SequentialCommandGroup(
                        new WaitCommand(waitAimTimer),
                        new horizontalBlockerCMD(sorterSb, blockerHFreePos).asProxy(),
                        new WaitCommand(artifacToArtifactTimer)
                )

        )

                ;
    }

    public Command startCMD() {

                return new SequentialCommandGroup(
                        new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                        new lateralBlockersCMD(sorterSb, 0, blockersUp)
                );
    }
}