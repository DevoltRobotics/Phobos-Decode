package org.firstinspires.ftc.teamcode.Utilities;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.horizontalBlockerCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Lifting.LiftingSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.turretToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

public abstract class OpModeCommand extends OpMode {

    public Follower follower;

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

        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        register(
                pedroSb = new PedroSubsystem(follower, telemetry),
                intakeSb = new IntakeSubsystem(hardwareMap),
                sorterSb = new SorterSubsystem(hardwareMap, telemetry),
                turretSb = new TurretSubsystem(hardwareMap, telemetry, true),
                //liftingSubsystem = new LiftingSubsystem(hardwareMap),
                shooterSb = new ShooterSubsystem(hardwareMap, telemetry, true),
                visionSb = new VisionSubsystem(hardwareMap)
        );

        imu = hardwareMap.get(IMU.class, "imu");

        initialize();
    }

    public Command cyclesShootCMD() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new turretToBasketCMD(turretSb, visionSb),
                        new shooterToBasketCMD(shooterSb, visionSb, follower)
                ),

                new WaitCommand(500),

                new ParallelCommandGroup(
                        new moveIntakeCMD(intakeSb, 1)
                        //new horizontalBlockerCMD(so, blockerHFreePos)
                ),

                new WaitCommand(200)


        );
    }

    public void initImu() {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    @Override
    public void init_loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        run();
    }

    public void stop() {
        reset();
    }

    public abstract void initialize();

}