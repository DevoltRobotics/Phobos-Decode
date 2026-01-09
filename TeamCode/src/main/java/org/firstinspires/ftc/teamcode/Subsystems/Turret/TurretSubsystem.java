package org.firstinspires.ftc.teamcode.Subsystems.Turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

    public Telemetry telemetry;

    public Follower follower;

    public Aliance alliance;


    public static double Minimum = 0.037;
    public static double MinimumEnc = 0.06;


    public static double capstanRatio = (double) 58 / 182;

    public static double ticktsToDegrees = (double) 360 / 8192;

    public static double furtherCorrectionAuto = 2;

    public static double furtherCorrection = 3;

    public static double manualIncrement = 2.5;

    public static int upperLimit = 110;

    public static int lowerLimit = -110;

    public double encoderP = 0;

    public double turretPRelative = 0;

    public double turretTarget = 0;

    double robotToGoalAngle;
    double turretToGoalAngle;

    public double distanceToGoal = 0;

    double goalX, goalY;

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
            setGoalPos(143, 143);
        } else if (alliance == Aliance.BLUE) {
            setGoalPos(0, 143);
        } else {
            setGoalPos(143, 143);
        }

        turretPid.setCoefficients(TurretSubsystem.principalTurretCoeffs);
        turretPid.setMinimumOutput(TurretSubsystem.MinimumEnc);


    }

    public void setTurretPower(double power) {
        if ((turretPRelative < -turretPRelative && power > 0) || (turretPRelative > upperLimit && power < 0)) {
            turretS1.setPower(0);
            turretS2.setPower(0);

        } else {
            turretS1.setPower(power);
            turretS2.setPower(power);
        }
    }

    @Override
    public void periodic() {

        Pose robotPos = follower.getPose();

        double dx = goalX - robotPos.getX();
        double dy = goalY - robotPos.getY();

        robotToGoalAngle = Math.toDegrees(Math.atan2(dy, dx));//direction
        distanceToGoal = Math.hypot(dx,dy);//magnitude

        turretToGoalAngle = AngleUnit.normalizeDegrees(Math.toDegrees(robotPos.getHeading()) - robotToGoalAngle);

        encoderP = turretE.getCurrentPosition();

        turretPRelative = AngleUnit.normalizeDegrees((encoderP * capstanRatio * ticktsToDegrees));

        FtcDashboard.getInstance().getTelemetry().addData("turret power", turretS1.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("turret angle", turretPRelative);
        FtcDashboard.getInstance().getTelemetry().addData("turret to goal angle", getTurretToGoalAngle());
        FtcDashboard.getInstance().getTelemetry().addData("distance to goal", getDistanceToGoal() );

    }

    public void setTarget (int target){
        turretTarget = target;

    }

    public double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public double getRobotToGoalAngle() {
        return robotToGoalAngle;
    }

    public double getTurretToGoalAngle() {
        return turretToGoalAngle;
    }

    public double getDistanceToGoal(){
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
