package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrectionAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

@Config
public class turretToBasketFurtherAutoCMD extends CommandBase {

    private final TurretSubsystem turretSb;
    private final VisionSubsystem visionSb;

    ElapsedTime deadTimer;

    public turretToBasketFurtherAutoCMD(TurretSubsystem turretSb, VisionSubsystem vSb) {
        this.turretSb = turretSb;
        this.visionSb = vSb;


        deadTimer = new ElapsedTime();

        addRequirements(turretSb);
    }


    @Override
    public void execute() {
        Double tA = visionSb.getAllianceTA();

        Double tX = visionSb.getAllianceTX();

        if (tX != null && tA != null) {

            if (tA < 50) {
                if (!visionSb.isAuto) {
                    switch (visionSb.alliance) {
                        case RED:
                            tX -= furtherCorrectionAuto;
                            break;

                        case BLUE:
                            tX += furtherCorrectionAuto;
                            break;
                    }
                }
            }

            deadTimer.reset();

        }
    }

    @Override
    public boolean isFinished() {
        return deadTimer.seconds() > 1 || visionSb.result.getTx() > 2;
    }
}



