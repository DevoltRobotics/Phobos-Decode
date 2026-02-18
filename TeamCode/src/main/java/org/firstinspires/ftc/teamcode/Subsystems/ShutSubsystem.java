package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.capstanRatio;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.ffValue;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.lowerLimit;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.principalTurretCoeffs;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.ticktsToDegrees;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.upperLimit;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.GOAL_POSE_BLUE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.GOAL_POSE_RED;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.gethoodTicksFromDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;

public class ShutSubsystem extends SubsystemBase {

    public Follower follower;

    //SHOOTER
    DcMotorEx shooterM;

    public static double shooterkV = 0.000525;
    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(0.01, 0.0, 0.0, 0);
    PIDFController shooterController;
    Telemetry telemetry;

    public double shooterTarget = 0;

    public double error = 0;

    //HOOD

    Servo hoodS;

    //TURRET

    public DcMotorEx turretM;

    public Aliance alliance;

    public PIDFController turretPid;

    public static double turretP = 0;

    public double turretTarget = 0;

    double turretPower = 0;

    public double robotToGoalAngle;
    public double turretToGoalAngle;

    public double distanceToGoal = 0;

    double goalX, goalY;

    public boolean realIsManual;

    public static double turretEndPose = 0;

    boolean isAuto;


    public ShutSubsystem(HardwareMap hMap, Telemetry telemetry, Follower follower, Aliance alliance, boolean isAuto) {

        //SHOOTER
        shooterM = hMap.get(DcMotorEx.class, "shdown");

        shooterM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterController = new PIDFController(shooterCoeffs);

        //HOOD
        hoodS = hMap.servo.get("hoodS");

        //TURRET
        turretM = hMap.get(DcMotorEx.class, "trtM");

        this.follower = follower;

        this.isAuto = isAuto;

        if (isAuto) {
            this.turretM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.turretM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.telemetry = telemetry;
        this.alliance = alliance;

        if (alliance.equals(Aliance.RED)){
            goalX = GOAL_POSE_RED.getX();
            goalY = GOAL_POSE_RED.getY();
        }else{
            goalX = GOAL_POSE_BLUE.getX();
            goalY = GOAL_POSE_BLUE.getX();

        }

        turretPid = new PIDFController(principalTurretCoeffs);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

        //SHOOTER
        shooterController.setCoefficients(shooterCoeffs);

        double motorVel = shooterM.getVelocity();

        shooterController.setSetPoint(shooterTarget);

        double shooterTargetPwr = (shooterkV * shooterTarget) + shooterController.calculate(motorVel);

        shooterM.setPower(shooterTargetPwr);

        error = shooterTarget - motorVel;

        FtcDashboard.getInstance().getTelemetry().addData("shooterError", error);

        //TURRET

        Pose robotPos = follower.getPose();

        double dx = goalX - robotPos.getX();
        double dy = goalY - robotPos.getY();

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy, dx));//direction

        distanceToGoal = Math.hypot(dx, dy);//magnitude

        turretToGoalAngle = AngleUnit.normalizeDegrees(Math.toDegrees(robotPos.getHeading()) - robotToGoalAngle);

        turretP = (turretM.getCurrentPosition() * capstanRatio * ticktsToDegrees);

        turretEndPose = turretP;

        double target = Range.clip(turretTarget, lowerLimit, upperLimit);

        double ff = Math.signum(turretTarget - turretP) * ffValue;

        turretPower = Range.clip(turretPid.calculate(turretP, target), -1, 1);

        turretM.setPower(turretPower + ff);

        turretPid.setCoefficients(principalTurretCoeffs);
        //turretPid.setMinimumOutput(MinimumEnc);

        FtcDashboard.getInstance().getTelemetry().addData("turretTarget", turretTarget);

        FtcDashboard.getInstance().getTelemetry().addData("turret angle", turretP);
    }

    public void setTurretTarget(double target){
        turretTarget = target;

    }

    public void setShooterTarget(double target){
        shooterTarget = target;

    }

    public void setHoodPose(double angle){
        hoodS.setPosition(gethoodTicksFromDegrees(angle));

    }
}
