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
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.moveIntakeAutonomousCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToBasketCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.shooterToVelCMD;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.postSorterCmd;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.preSorterCmd;
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

@Config
public abstract class OpModeCommand extends OpMode {

    public Follower follower;

    public final Aliance currentAliance;

    public final boolean isAuto;

    public TelemetryManager telemetryM;

    public VisionSubsystem visionSb;
    public PedroSubsystem pedroSb;
    public IntakeSubsystem intakeSb;
    public SorterSubsystem sorterSb;
    public SensorsSubsystem sensorsSb;
    //public LiftingSubsystem liftingSb;
    public ShooterSubsystem shooterSb;
    public TurretSubsystem turretSb;

    public IMU imu;


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
                turretSb = new TurretSubsystem(hardwareMap, follower, telemetry, intakeSb.intakeM, currentAliance),
                //liftingSb = new LiftingSubsystem(hardwareMap),
                shooterSb = new ShooterSubsystem(hardwareMap, telemetry)
        );


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
        //telemetry.addData("PATTERN", visionSb.alliance);

        if (isAuto) {
            run();

        }

    }


    @Override
    public void loop() {
        telemetry.addData("Heading", Math.toDegrees(follower.poseTracker.getPose().getHeading()));

        //telemetryM.update(telemetry);
        FtcDashboard.getInstance().getTelemetry().update();
        run();
    }

    public void stop() {
        reset();
    }

    public abstract void initialize();


    public Command startCMD() {
        return new SequentialCommandGroup(
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                new lateralBlockersCMD(sorterSb, 0, blockersUp),
                new rampCMD(sorterSb, upRampPos)
        );
    }

    public Command sorter3CMD(PathChain shootPath) {
        return new SequentialCommandGroup(
                new moveIntakeAutonomousCMD(intakeSb, 0),
                new WaitCommand(100),

                new preSorterCmd(sorterSb, sensorsSb, visionSb, 0.3),

                new moveIntakeAutonomousCMD(intakeSb, 1, 0.7),

                new shooterToBasketCMD(shooterSb, turretSb),

                new WaitCommand(400),

                new InstantCommand(
                        () -> pedroSb.follower.setMaxPower(0.9)
                ),

                new ParallelRaceGroup(

                        new turretToBasketCMD(turretSb),
                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(shootPath),

                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        new moveIntakeAutonomousCMD(intakeSb, -0.6),
                                        new WaitCommand(150),
                                        new postSorterCmd(sorterSb, sensorsSb, visionSb),
                                        new moveIntakeAutonomousCMD(intakeSb, 1)

                                )
                        ))
        );
    }

    public Command shootThreeCMD() {
        return new ParallelDeadlineGroup(

                new WaitCommand(1500), // deadline

                new SequentialCommandGroup(
                        new horizontalBlockerCMD(sorterSb, blockerHFreePos),
                        new WaitCommand(1000),
                        new lateralBlockersCMD(sorterSb, blockersUp, blockersUp)
                ),

                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new moveIntakeAutonomousCMD(intakeSb, 1)
                ),

                new turretToBasketCMD(turretSb)
        );
    }

    public Command stopShootCMD() {
        return new SequentialCommandGroup(
                new moveIntakeAutonomousCMD(intakeSb, 0),
                new shooterToVelCMD(shooterSb, 0),
                new turretToPosCMD(turretSb, 0.0),
                new horizontalBlockerCMD(sorterSb, blockerHHidePos),
                new lateralBlockersCMD(sorterSb, 0, 0)
        );
    }
}