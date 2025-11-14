package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;

@Config
public class turretToBasketAngle extends CommandBase {

    public static PIDFCoefficients llPidCoeffs = new PIDFCoefficients(0.043, 0.0, 0.0001, 0);

    PIDFController llPidf;

    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystem visionSubsystem;

    private ElapsedTime timer;

    public static double redGoalAngle = 60;

    public static double blueGoalAngle = 150;

    private double staticTarget;

    private boolean hasTargeted = false;

    Follower follower;

    public turretToBasketAngle(TurretSubsystem turretSb, VisionSubsystem vSb, Follower fllw) {
        turretSubsystem = turretSb;
        visionSubsystem = vSb;

        addRequirements(turretSubsystem);
    }


    @Override
    public void initialize() {
        timer = new ElapsedTime();

        llPidf = new PIDFController(llPidCoeffs);
        turretSubsystem.isTurretManual = false;
    }

    @Override
    public void execute() {
        llPidf.setCoefficients(llPidCoeffs);
        llPidf.setSetPoint(0);
        llPidf.setTolerance(0.1);

        Double tA = visionSubsystem.getAllianceTA();

        Double tX = visionSubsystem.getAllianceTX();

        if(tX != null && tA != null) {
            if (tA < 50) {
                tX += 5.5;
            }

            double llTargt = llPidf.calculate(tX);
            turretSubsystem.turretTarget -= llTargt;

            FtcDashboard.getInstance().getTelemetry().addData("llTurretTarget", llTargt);

            hasTargeted = true;
        }else {
            double robotHeading = Math.toDegrees(follower.getHeading());

            if (visionSubsystem.alliance == Alliance.RED) {
                staticTarget = robotHeading - redGoalAngle;

            } else {
                turretSubsystem.turretTarget = robotHeading - blueGoalAngle;
            }

        }
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() > 1.5;
    }
}
