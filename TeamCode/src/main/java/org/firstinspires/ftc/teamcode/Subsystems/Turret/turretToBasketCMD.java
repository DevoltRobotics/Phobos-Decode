package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static android.graphics.Color.RED;
import static org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem.limelightTaRatio;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;

@Config
public class turretToBasketCMD extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystem visionSubsystem;

    boolean isAuto;
    public turretToBasketCMD(TurretSubsystem turretSb, VisionSubsystem vSb, boolean isAuto) {
        turretSubsystem = turretSb;
        visionSubsystem = vSb;

        this.isAuto = isAuto;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        Double tA = visionSubsystem.getAllianceTA();

        Double tX = visionSubsystem.getAllianceTX();

        if(tX != null && tA != null) {
            if (tA < 50) {
                if (!isAuto) {
                    switch (visionSubsystem.alliance) {

                        case RED:
                            tX += 5.5;
                            break;

                        case BLUE:
                            tX -= 5.5;
                            break;
                    }
                }
            }

            double llTargt = turretSubsystem.llPidf.calculate(tX);
            turretSubsystem.turretTarget -= llTargt;

            FtcDashboard.getInstance().getTelemetry().addData("llTurretTarget", llTargt);

        }
    }

}
