package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockerHHidePos;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.upRampPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.rampCMD;
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

    GoBildaPinpointDriver pinpoint;


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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );


        register(
                visionSb = new VisionSubsystem(hardwareMap, currentAliance, isAuto, pinpoint)

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
}