package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHFreePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;

import android.os.Trace;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Lifting.LiftingSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.postSorterCmd;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.preSorterTeleopCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.lateralBlockersCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.function.BooleanSupplier;

public abstract class OpModeCommand extends OpMode {

    public Follower follower;
    private final ElapsedTime timer = new ElapsedTime();

    private double lastTime = 0;
    public final Alliance currentAlliance;

    public final boolean isAuto;
    public final boolean closeAuto;


    public VisionSubsystem visionSb;
    public PedroSubsystem pedroSb;
    public IntakeSubsystem intakeSb;
    public SorterSubsystem sorterSb;
    public SensorsSubsystem sensorsSb;
    public ShooterSubsystem shooterSb;

    public boolean isLifting = false;

    public OpModeCommand(Alliance alliance, boolean isAuto, boolean closeAuto) {
        this.currentAlliance = alliance;
        this.isAuto = isAuto;
        this.closeAuto = closeAuto;
    }

    //reinicia la lista de comandos
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    //corre el scheduler
    public void run() {
        //run();

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
                visionSb = new VisionSubsystem(hardwareMap, telemetry, currentAlliance, isAuto, closeAuto)
        );

        follower = Constants.createFollower(hardwareMap, visionSb.ll);

        follower.update();

        //Drawing.init();

        register(
                visionSb = new VisionSubsystem(hardwareMap, telemetry, currentAlliance, isAuto, closeAuto),
                pedroSb = new PedroSubsystem(follower, telemetry, currentAlliance),
                intakeSb = new IntakeSubsystem(hardwareMap),
                sorterSb = new SorterSubsystem(hardwareMap, telemetry),
                sensorsSb = new SensorsSubsystem(hardwareMap, telemetry),
                shooterSb = new ShooterSubsystem(hardwareMap, telemetry, follower, currentAlliance, isAuto)
        );

        if (isAuto && !closeAuto) {
            visionSb.setLLState(VisionSubsystem.llState.artifact);
        } else {
            visionSb.setLLState(VisionSubsystem.llState.posEstimate);
        }

        //imu = hardwareMap.get(IMU.class, "imu");

        initialize();
    }

    @Override
    public void init_loop() {
        if (isAuto) {
            CommandScheduler.getInstance().run();

        }
    }

    double dt;

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        run();

        dt = timer.seconds() - lastTime;

        telemetry.addData("deltaT", dt);

        PanelsTelemetry.INSTANCE.getFtcTelemetry().update();

        lastTime = timer.seconds();

        follower.update();

        /*Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

         */
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

    public Command shootThreeSpamerFarCMD() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHFreePos)),
                new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),
                new WaitCommand(900) // deadline


        );
    }


    public Command shootThreeSpamerCloseCMD() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHFreePos)),
                new InstantCommand(() -> intakeSb.setIntakePower(1, 1)),
                new WaitCommand(800) // deadline


        );
    }

    public Command stopShootCMD(boolean isSorter) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intakeSb.setIntakePower(0, 0)),

                new InstantCommand(() -> sorterSb.setHorizontalPos(blockerHHidePos)),

                new ConditionalCommand(
                        new lateralBlockersCMD(sorterSb, 0, 0),
                        new lateralBlockersCMD(sorterSb, blockersUp, 0),
                        () -> isSorter
                ),

                //new InstantCommand(() -> shooterSb.setOffTarget()),
                new InstantCommand(() -> shooterSb.setTurretTarget(0))

        );

    }

    public Command shootThreesorterCMD(int timer) {
        return new SequentialCommandGroup(
                new preSorterTeleopCMD(sorterSb, sensorsSb, 300),

                new WaitCommand(timer), // deadline

                new postSorterCmd(sorterSb, sensorsSb)
        );
    }

    public Command switchPatternTarget() {
        return new InstantCommand(
                () -> {
                    switch (sensorsSb.teleOpPattern) {
                        case PPG:
                            sensorsSb.teleOpPattern = Pattern.PGP;
                            break;
                        case PGP:
                            sensorsSb.teleOpPattern = Pattern.GPP;
                            break;
                        case GPP:
                            sensorsSb.teleOpPattern = Pattern.PPG;
                            break;
                    }
                }
        );
    }
}
