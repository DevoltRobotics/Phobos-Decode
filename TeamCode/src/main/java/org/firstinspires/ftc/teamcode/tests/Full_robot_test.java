package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem.shooterCoeffs;
import static org.firstinspires.ftc.teamcode.Subsystems.SorterSubsystem.SorterSubsystem.blockersUp;
import static org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem.limelightTaRatio;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint0;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout0;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout4;

import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout5;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;

import com.pedropathing.follower.Follower;

@TeleOp
@Config
public class Full_robot_test extends OpMode {

    public static double provitionalShooterTarget = 1400;

    CRServo capstaneSr;

    DcMotorEx fr;
    DcMotorEx br;
    DcMotorEx bl;
    DcMotorEx fl;

    DcMotorEx shooterMUp;
    DcMotorEx shooterMDown;
    DcMotorEx intakeM;

    Servo blockerR;
    Servo blockerL;

    Servo blockerH;

    Servo hood;

    public InterpLUT vsFunc = new InterpLUT();

    Limelight3A limelight3A;

    double tArea = shoootVsint0 + 1;

    PIDFController shooterController = new PIDFController(shooterCoeffs);

    ElapsedTime toggleTimer = new ElapsedTime();
    public boolean toggleShooterP;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        startingPose = new Pose(0, 0,0 );
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //pinpoint = hardwareMap.get(SensorGoBildaPinpoint.class, "pinpoint");

        capstaneSr = hardwareMap.get(CRServo.class, "trt");

        blockerH = hardwareMap.get(Servo.class, "blcH");

        fr = hardwareMap.get(DcMotorEx.class, "fr");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        fl = hardwareMap.get(DcMotorEx.class, "fl");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMUp = hardwareMap.get(DcMotorEx.class, "shup");
        shooterMDown = hardwareMap.get(DcMotorEx.class, "shdown");
        intakeM = hardwareMap.get(DcMotorEx.class, "in");

        hood = hardwareMap.get(Servo.class, "hood");

        blockerR = hardwareMap.get(Servo.class, "blcR");
        blockerL = hardwareMap.get(Servo.class, "blcL");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        limelight3A.pipelineSwitch(0);
        limelight3A.start();

        shooterMUp.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        vsFunc.add(shoootVsint0, shoootVsout0);
        vsFunc.add(shoootVsint1, shoootVsout1);
        vsFunc.add(shoootVsint2, shoootVsout2);
        vsFunc.add(shoootVsint3, shoootVsout3);
        vsFunc.add(shoootVsint4, shoootVsout4);
        vsFunc.add(shoootVsint5, shoootVsout5);
        vsFunc.createLUT();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );

        //Slow Mode

        slowMode = gamepad1.right_trigger > 0.5;
        //Optional way to change slow mode strength

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

        double motorVel = shooterMUp.getVelocity();

        shooterController.setCoefficients(shooterCoeffs);

        LLResult result = limelight3A.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                tArea = Range.clip(result.getTa() * limelightTaRatio, shoootVsint0 + 0.1, shoootVsint5 - 0.1);
            }
        }

        double shooterTarget = vsFunc.get(tArea);

        shooterController.setSetPoint(provitionalShooterTarget);

        double velocityError = provitionalShooterTarget - motorVel;
        double shooterTargetPwr = (ShooterSubsystem.shooterkV * provitionalShooterTarget) + shooterController.calculate(motorVel);

        if (gamepad2.dpad_up && toggleTimer.seconds() > 0.5) {
            toggleShooterP = !toggleShooterP;
            toggleTimer.reset();
        }

        if (toggleShooterP) {
            shooterMUp.setPower(shooterTargetPwr);
            shooterMDown.setPower(shooterTargetPwr);
        } else {
            shooterMUp.setPower(0);
            shooterMDown.setPower(0);
        }

        if (gamepad2.right_bumper) {
            capstaneSr.setPower(1);

        } else if (gamepad2.left_bumper) {
            capstaneSr.setPower(-1);

        } else {
            capstaneSr.setPower(0);

        }

        if (gamepad2.right_trigger > 0.5){
            intakeM.setPower(-1);
        }else if (gamepad2.left_trigger > 0.5){
            intakeM.setPower(1);
        }else {
            intakeM.setPower(0);
        }

        if (gamepad2.a) {
            hood.setPosition(0.47);

        } else if (gamepad2.b) {
            hood.setPosition(0.6);

        }

        if (gamepad2.dpad_right){
            blockerH.setPosition(0.45);

        }else if (gamepad2.dpad_left) {
            blockerH.setPosition(0.75);
        }

        if (gamepad2.y) {
            blockerR.setPosition(0.5 + blockersUp);
            blockerL.setPosition(0.5 - blockersUp);
        } else if (gamepad2.x) {
            blockerR.setPosition(0.5);
        }

        telemetry.addData("shooterVelocity", motorVel);
        telemetry.addData("shooterTarget", provitionalShooterTarget);
        telemetry.addData("toggle", toggleShooterP);
        telemetry.addData("vError", velocityError);
        telemetry.addData("ta", tArea);

        telemetry.addData("servoPosition", blockerH.getPosition());

        telemetry.update();

    }
}
