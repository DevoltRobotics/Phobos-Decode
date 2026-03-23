package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
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
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

public abstract class OpModeCommand extends OpMode {

    public Follower follower;

    public final Aliance currentAliance;

    public final boolean isAuto;

    public VisionSubsystem visionSb;
    public PedroSubsystem pedroSb;
    public IntakeSubsystem intakeSb;
    public SorterSubsystem sorterSb;
    public SensorsSubsystem sensorsSb;
    public LiftingSubsystem liftingSb;
    public ShooterSubsystem shooterSb;

    public TelemetryManager telemetryM;

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

        telemetry = new JoinedTelemetry(
                telemetry,
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        register(
                visionSb = new VisionSubsystem(hardwareMap, telemetry, currentAliance, isAuto, follower)

        );

        follower = Constants.createFollower(hardwareMap, visionSb.ll);

        follower.update();

        register(
                pedroSb = new PedroSubsystem(follower, telemetry),
                intakeSb = new IntakeSubsystem(hardwareMap),
                sorterSb = new SorterSubsystem(hardwareMap, telemetry),
                sensorsSb = new SensorsSubsystem(hardwareMap, telemetry),
                //turretSb = new TurretSubsystem(hardwareMap, follower, telemetry, currentAliance, isAuto),
                liftingSb = new LiftingSubsystem(hardwareMap),
                //shooterSb = new ShooterSubsystem(hardwareMap, telemetry),
                shooterSb = new ShooterSubsystem(hardwareMap, telemetry, follower, currentAliance, isAuto)
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
        PanelsTelemetry.INSTANCE.getFtcTelemetry().update();
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

                )
        );
    }


    public Command shootThreeSpamerCloseCMD() {
        return new ParallelDeadlineGroup(

                new WaitCommand(1700), // deadline

                new SequentialCommandGroup(
                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                        new moveIntakeAutonomousCMD(intakeSb, 1)

                )

        );
    }

    public Command stopShootCMD(boolean isSorter) {
        return new SequentialCommandGroup(
                new moveIntakeAutonomousCMD(intakeSb, 0, 0),

                new horizontalBlockerCMD(sorterSb, blockerHHidePos),

                new ConditionalCommand(
                        new lateralBlockersCMD(sorterSb, 0, 0),
                        new lateralBlockersCMD(sorterSb, blockersUp, 0),
                        () -> isSorter
                )
        );

    }
}