package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Lifting.LiftingSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToPosCMD;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.List;

@Config
public abstract class OpModeCommand extends OpMode {

    List<LynxModule> allhubs;

    public Follower follower;

    public final Aliance currentAliance;

    public final boolean isAuto;

    public TelemetryManager telemetryM;

    public VisionSubsystem visionSb;
    public PedroSubsystem pedroSb;
    public IntakeSubsystem intakeSb;
    public SorterSubsystem sorterSb;
    public SensorsSubsystem sensorsSb;
    public LiftingSubsystem liftingSb;
    public ShooterSubsystem shooterSb;
    public TurretSubsystem turretSb;

    public OpModeCommand(Aliance alliance, boolean isAuto) {
        this.currentAliance = alliance;
        this.isAuto = isAuto;
    }

    //reinicia la lista de comandos
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    //corre el scheduler
    public void run() {
        run();

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
        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        register(
                visionSb = new VisionSubsystem(hardwareMap, telemetry, currentAliance, isAuto, follower)

        );

        follower = Constants.createFollower(hardwareMap, visionSb.ll);

        follower.update();

        register(
                pedroSb = new PedroSubsystem(follower, telemetry, telemetryM),
                intakeSb = new IntakeSubsystem(hardwareMap),
                sorterSb = new SorterSubsystem(hardwareMap, telemetry),
                sensorsSb = new SensorsSubsystem(hardwareMap, telemetry),
                turretSb = new TurretSubsystem(hardwareMap, follower, telemetry, currentAliance, isAuto),
                liftingSb = new LiftingSubsystem(hardwareMap),
                shooterSb = new ShooterSubsystem(hardwareMap, telemetry)
        );


        //imu = hardwareMap.get(IMU.class, "imu");

        initialize();
    }

    @Override
    public void init_loop() {
        if (isAuto) {
            CommandScheduler.getInstance().run();

        }
    }


    @Override
    public void loop() {

        CommandScheduler.getInstance().run();
        run();

        telemetry.addData("Heading", Math.toDegrees(follower.poseTracker.getPose().getHeading()));

        //telemetryM.update(telemetry);
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public void stop() {
        reset();
    }

    public abstract void initialize();


    public Command startCMD() {
        return new SequentialCommandGroup(
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                new lateralBlockersCMD(sorterSb, blockersUp, 0),
                new rampCMD(sorterSb, upRampPos)
        );
    }

    public Command shootThreeSpamerFarCMD(double shooterVel) {
        return new ParallelDeadlineGroup(

                new WaitCommand(1300), // deadline

                new SequentialCommandGroup(
                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                        new moveIntakeAutonomousCMD(intakeSb, 0.9, 0.75)

                ),
                new turretToBasketCMD(turretSb, visionSb),
                new shooterToBasketCMD(shooterSb, visionSb, turretSb)

        );
    }


    public Command shootThreeSpamerCloseCMD() {
        return new ParallelDeadlineGroup(

                new WaitCommand(1700), // deadline

                new SequentialCommandGroup(
                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                        new moveIntakeAutonomousCMD(intakeSb, 1)

                ),

                new turretToBasketCMD(turretSb, visionSb),
                new shooterToBasketCMD(shooterSb, visionSb, turretSb)
        );
    }

    public Command stopShootCMD(boolean isSorter) {
        return new SequentialCommandGroup(
                new moveIntakeAutonomousCMD(intakeSb, 0, 0),
                new shooterToVelCMD(shooterSb, 800),
                new turretToPosCMD(turretSb, 0.0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new ConditionalCommand(
                        new lateralBlockersCMD(sorterSb, 0, 0),
                        new lateralBlockersCMD(sorterSb, blockersUp, 0),
                        () -> isSorter)

        );
    }
}