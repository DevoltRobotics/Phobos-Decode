package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.llPidCoeffs;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.manualIncrement;

import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;

@Configurable
@Config
public class turretManaulCMD extends CommandBase {

    private final TurretSubsystem turretSubsystem;

    private final VisionSubsystem visionSubsystem;

    private Gamepad gamepad;

    Follower follower;

    double newTurretTargetRelative = 0;

    private double goalY = 144;
    private double goalX = 0;

    public turretManaulCMD(TurretSubsystem turretSb, VisionSubsystem visionSb, Follower fllw, Gamepad gamepad) {
        turretSubsystem = turretSb;
        this.gamepad = gamepad;

        visionSubsystem = visionSb;
        follower = fllw;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Double tA = visionSubsystem.getAllianceTA();
        Double tX = visionSubsystem.getAllianceTX();

        double robotHeading = Math.toDegrees(follower.poseTracker.getPose().getHeading());

        if(tX != null && tA != null) {
            if (tA < 50) {

                    switch (visionSubsystem.alliance) {
                        case RED:
                            tX += 5.5;
                            break;

                        case BLUE:
                            tX -= 5.5;
                            break;
                    }
            }

            double llTargt = turretSubsystem.llPidf.calculate(tX);
            turretSubsystem.turretTarget -= llTargt;

            if (Math.abs(tX) < 0.5){
                newTurretTargetRelative = (turretSubsystem.turretPRelative - robotHeading);

                turretSubsystem.newTurretTarget = robotHeading + newTurretTargetRelative;
            }

            FtcDashboard.getInstance().getTelemetry().addData("llTurretTarget", llTargt);

        }

    }
}
