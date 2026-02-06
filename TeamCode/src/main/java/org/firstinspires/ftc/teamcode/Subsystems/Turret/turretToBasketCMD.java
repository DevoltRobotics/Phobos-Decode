package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

@Config
public class turretToBasketCMD extends CommandBase {

    TurretSubsystem turretSb;

    VisionSubsystem visionSb;

    private final ElapsedTime waitAimTimer;

    public turretToBasketCMD(TurretSubsystem turretSb, VisionSubsystem visionSb) {

        this.turretSb = turretSb;
        this.visionSb = visionSb;

        waitAimTimer = new ElapsedTime();
        addRequirements(turretSb);
    }

    @Override
    public void execute() {

        Double tA = visionSb.getAllianceTA();

        Double tX = visionSb.getAllianceTX();

        if (tX != null && tA != null) {

            if (tA < 50) {
                switch (visionSb.alliance) {
                    case RED:
                        tX += furtherCorrection;
                        break;

                    case BLUE:
                        tX -= furtherCorrection;
                        break;
                }

            }

            double llTarget = turretSb.llPidf.calculate(tX);
            turretSb.turretTarget -= llTarget;

            turretSb.telemetry.addData("llTarget", llTarget);

        }
    }

}



