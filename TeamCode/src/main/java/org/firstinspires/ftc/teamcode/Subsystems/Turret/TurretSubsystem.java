package org.firstinspires.ftc.teamcode.Subsystems.Turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;

@Config
public class TurretSubsystem extends SubsystemBase {

    public static double trtTarget = 60;

    public DcMotorEx turretM;

    public Telemetry telemetry;

    public Follower follower;

    public Aliance alliance;

    public static PIDFCoefficients principalTurretCoeffs = new PIDFCoefficients(0.02, 0.0, 0.0015, 0.00049);

    public PIDFController principalTurretPid;

    public static PIDFCoefficients secondaryTurretCoeffs = new PIDFCoefficients(0.035, 0.0, 0.0011, 0.00049);

    public PIDFController secondaryTurretPid;

    public static PIDFCoefficients llPidCoeffs = new PIDFCoefficients(0.1, 0.0, 0.008, 0);
    public PIDFController llPidf;

    public static double turretPidSwitch = 10; // start SMALL

    public static boolean useSecondaryPID = true;
    public static double minimunPower = 0.03;

    public static double MinimumEnc = 0.07;

    public static double capstanRatio = 0.331; //0.331

    public static double ticktsToDegrees = (double) 360 / 537.7;

    public static double furtherCorrection = 5;

    public static int upperLimit = 110;

    public static int lowerLimit = -110;

    public static double turretP = 0;

    public double turretTarget = 0;

    double turretPower = 0;

    public double robotToGoalAngle;
    public double turretToGoalAngle;

    double goalAngleRad;
    public double distanceToGoal = 0;

    public static double aallY = 138;
    public static double redX = 138;
    public static double blueX = 6;

    double goalX, goalY;

    public boolean realIsManual;

    public static int manualIncrement = 5;

    public static double turretEndPose = 0;

    boolean isAuto;


    public void setGoalPos(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public TurretSubsystem(HardwareMap hMap, Follower follower, Telemetry telemetry, Aliance alliance, boolean isAuto) {
        turretM = hMap.get(DcMotorEx.class, "trtM");

        turretM.setDirection(DcMotorSimple.Direction.REVERSE);
        this.follower = follower;

        this.isAuto = isAuto;

        /*if (isAuto) {

            this.turretM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.turretM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

         */

        this.turretM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turretM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
        this.alliance = alliance;

        if (alliance == Aliance.RED) {
            setGoalPos(redX, aallY);

             goalAngleRad = Math.toRadians(38);
        } else if (alliance == Aliance.BLUE) {
            setGoalPos(blueX, aallY);

            goalAngleRad = Math.toRadians(142);
        } else {
            setGoalPos(redX, aallY);
        }

        principalTurretPid = new PIDFController(principalTurretCoeffs);
        secondaryTurretPid = new PIDFController(secondaryTurretCoeffs);

        llPidf = new PIDFController(llPidCoeffs);



    }

    @Override
    public void periodic() {
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

        principalTurretPid.setMinimumOutput(minimunPower);
        secondaryTurretPid.setMinimumOutput(minimunPower);

        if (Math.abs(error) > turretPidSwitch || !useSecondaryPID) {
            turretPower = Range.clip(principalTurretPid.calculate(turretP, target), -1, 1);
        }else{
            turretPower = Range.clip(secondaryTurretPid.calculate(turretP, target), -1, 1);

        }
        turretM.setPower(turretPower);

        /*principalTurretPid.setCoefficients(principalTurretCoeffs);
        secondaryTurretPid.setCoefficients(secondaryTurretCoeffs);


         */
        FtcDashboard.getInstance().getTelemetry().addData("turretTarget", error);
        FtcDashboard.getInstance().getTelemetry().addData("turretError", turretTarget);

        FtcDashboard.getInstance().getTelemetry().addData("turret angle", turretP);
        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal());

    }

    public void setAutoTurretPos() {
        turretP = turretEndPose;

    }

    public void setTarget(double target) {
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

    public void resetEncoder() {
        turretM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
