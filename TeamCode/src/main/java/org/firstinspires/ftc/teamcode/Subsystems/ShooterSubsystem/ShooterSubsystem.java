package org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.GOAL_POSE_BLUE;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.GOAL_POSE_RED;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.capstanRatio;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.gethoodTicksFromDegrees;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.lowerLimit;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.minimunPower;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.principalTurretCoeffs;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.secondaryTurretCoeffs;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.shooterCoeffs;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.shooterkV;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.ticktsToDegrees;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.turretPidSwitch;
import static org.firstinspires.ftc.teamcode.Utilities.shooterConstants.upperLimit;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;

public class ShooterSubsystem extends SubsystemBase {

    public Follower follower;

    //SHOOTER
    DcMotorEx shooterM;

    PIDFController shooterController;
    Telemetry telemetry;

    public double shooterTarget = 0;

    public double error = 0;

    //HOOD

    Servo hoodS;

    //TURRET

    public DcMotorEx turretM;

    public Alliance alliance;

    public PIDFController turretPid;

    public PIDFController secondaryTurretPid;

    public static double turretP = 0;

    public double turretTarget = 0;

    double turretPower = 0;

    public double robotToGoalAngle;
    public double turretToGoalAngle;

    public double distanceToGoal = 0;

    double goalX, goalY;

    public static double turretEndPose = 0;

    boolean isAuto;

    public static boolean useSecondaryPID = false;


    public ShooterSubsystem(HardwareMap hMap, Telemetry telemetry, Follower follower, Alliance alliance, boolean isAuto) {

        //SHOOTER
        shooterM = hMap.get(DcMotorEx.class, "shM");

        shooterM.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterController = new PIDFController(shooterCoeffs);

        //HOOD
        hoodS = hMap.servo.get("hoodS");

        //TURRET
        turretM = hMap.get(DcMotorEx.class, "trtM");

        turretM.setDirection(DcMotorSimple.Direction.REVERSE);

        this.follower = follower;

        this.isAuto = isAuto;

        //if (isAuto) {
            this.turretM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.turretM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //}

        this.telemetry = telemetry;
        this.alliance = alliance;

        if (alliance.equals(Alliance.RED)){
            goalX = GOAL_POSE_RED.getX();
            goalY = GOAL_POSE_RED.getY();
        }else{
            goalX = GOAL_POSE_BLUE.getX();
            goalY = GOAL_POSE_BLUE.getX();

        }

        turretPid = new PIDFController(principalTurretCoeffs);
        secondaryTurretPid = new PIDFController(secondaryTurretCoeffs);
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

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("shooterTargetPwr", shooterTargetPwr);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("shooterTarget", shooterTarget);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("shooterError", error);
        //TURRET

        Pose robotPos = follower.getPose();

        double dx = goalX - robotPos.getX();
        double dy = goalY - robotPos.getY();

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy, dx));//direction

        distanceToGoal = Math.hypot(dx, dy);//magnitude

        turretToGoalAngle = AngleUnit.normalizeDegrees(Math.toDegrees(robotPos.getHeading()) - robotToGoalAngle);

        turretP =  (turretM.getCurrentPosition() * capstanRatio * ticktsToDegrees);

        turretEndPose = turretP;

        double target = Range.clip(turretTarget, lowerLimit, upperLimit);

        double error = target - turretP;

        turretPid.setMinimumOutput(minimunPower);
        secondaryTurretPid.setMinimumOutput(minimunPower);

        if (Math.abs(error) > turretPidSwitch || !useSecondaryPID) {
            turretPower = Range.clip(turretPid.calculate(turretP, target), -1, 1);
        }else{
            turretPower = Range.clip(secondaryTurretPid.calculate(turretP, target), -1, 1);

        }

        turretM.setPower(turretPower);

        /*principalTurretPid.setCoefficients(principalTurretCoeffs);
        secondaryTurretPid.setCoefficients(secondaryTurretCoeffs);

         */

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("turretError", error);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("turretTarget", turretTarget);

        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("turret angle", turretP);
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        PanelsTelemetry.INSTANCE.getFtcTelemetry().addData("distance to goal", getDistanceToGoal());


    }

    public void setShooterTarget(double target){
        shooterTarget = target;

    }

    public void setHoodPose(double angle){
        hoodS.setPosition(gethoodTicksFromDegrees(angle));

    }

    public void setTurretTarget(double target){
        turretTarget = target;

    }

    public double getRobotToGoalAngle() {
        return robotToGoalAngle;
    }

    public double getTurretToGoalAngle() {
        return turretToGoalAngle;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }


    public double getCurrentPosition() {
        return turretP;
    }

    public void resetTurret() {
        turretM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
