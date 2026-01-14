package org.firstinspires.ftc.teamcode.Subsystems.Turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public CRServo turretS1;
    public CRServo turretS2;

    public DcMotor turretE;

    public static PIDFCoefficients principalTurretCoeffs = new PIDFCoefficients(0.017, 0.0, 0.00056, 0.002);

    public PIDFController turretPid;

    public static PIDFCoefficients llPidCoeffs = new PIDFCoefficients(0.043, 0.0, 0.0001, 0);
    public PIDFController llPidf;


    public Telemetry telemetry;

    public Follower follower;

    public Aliance alliance;
    public static double Minimum = 0.037;
    public static double MinimumEnc = 0.06;


    public static double capstanRatio = (double) 58 / 182;

    public static double ticktsToDegrees = (double) 360 / 8192;

    public static double furtherCorrection = 3;

    public static int upperLimit = 110;

    public static int lowerLimit = -110;

    public double encoderP = 0;

    public static double turretPRelative = 0;

    public double turretTarget = 0;

    double turretPower = 0;

    public double robotToGoalAngle;
    public double turretToGoalAngle;

    public double distanceToGoal = 0;

    public static double aallY = 144;
    public static double redX = 144;
    public static double blueX = 0;

    public static double aallYFar = 144;
    public static double redXFar = 144;
    public static double blueXFar = 0;

    double goalX, goalY;

    public boolean realIsManual;

    public void setGoalPos(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public TurretSubsystem(HardwareMap hMap, Follower follower, Telemetry telemetry, DcMotor turretE, Aliance alliance) {
        turretS1 = hMap.get(CRServo.class, "trt1");
        turretS2 = hMap.get(CRServo.class, "trt2");

        this.follower = follower;

        this.turretE = turretE;

        this.turretE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.turretE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
        this.alliance = alliance;

        if (alliance == Aliance.RED) {
            setGoalPos(redX, aallY);
        } else if (alliance == Aliance.BLUE) {
            setGoalPos(blueX, aallY);
        } else {
            setGoalPos(redX, aallY);
        }

        turretPid = new PIDFController(principalTurretCoeffs);
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

        encoderP = turretE.getCurrentPosition();

        //turretPRelative = AngleUnit.normalizeDegrees((encoderP * capstanRatio * ticktsToDegrees));

        turretPRelative = (encoderP * capstanRatio * ticktsToDegrees);

        double target = Range.clip(turretTarget, lowerLimit, upperLimit);

        turretPower = turretPid.calculate(turretPRelative, target);

        turretS1.setPower(turretPower);
        turretS2.setPower(turretPower);

        turretPid.setCoefficients(principalTurretCoeffs);
        turretPid.setMinimumOutput(MinimumEnc);

        llPidf.setCoefficients(llPidCoeffs);
        llPidf.setSetPoint(0);

        FtcDashboard.getInstance().getTelemetry().addData("turretTarget", turretTarget);

        FtcDashboard.getInstance().getTelemetry().addData("turret angle", turretPRelative);
        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal());

    }

    public void setTarget(int target) {
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
        return turretPRelative;
    }

    public void resetEncoder() {
        turretE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
