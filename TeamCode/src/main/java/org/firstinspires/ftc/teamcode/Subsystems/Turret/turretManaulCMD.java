package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.llPidCoeffs;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.manualIncrement;

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

    private double targetGoalY = 144;
    private double targetGoalX = 0;

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

        Double robotHeading = Math.toDegrees(follower.poseTracker.getPose().getHeading());

        double deltaTurretAngle = turretSubsystem.turretP - robotHeading;

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

            FtcDashboard.getInstance().getTelemetry().addData("llTurretTarget", llTargt);

        }else {

            /*if (visionSubsystem.alliance == Alliance.RED) {
                staticTarget = robotHeading - redGoalAngle;

                double desiredTurretAngle =

            }

             */

        }

        if (gamepad.right_bumper) {
            turretSubsystem.turretTarget += manualIncrement;
        } else if (gamepad.left_bumper) {
            turretSubsystem.turretTarget -= manualIncrement;
        }

    }
}
