package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

public class turretToBasketCMD extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final Limelight3A limelight;

    private ElapsedTime stopperTimmer;

    double visionTarget = 0;

    double basketTarget = 0;
    public turretToBasketCMD(TurretSubsystem turretSb, VisionSubsystem vSb) {
        turretSubsystem = turretSb;
        visionSubsystem = vSb;

        limelight = visionSubsystem.limelight;

        addRequirements(turretSubsystem, visionSubsystem);
    }


    @Override
    public void initialize() {
        turretSubsystem.isTurretManual = false;

        stopperTimmer.reset();
    }

    @Override
    public void execute() {

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                visionTarget = turretSubsystem.turretvsFunc.get(result.getTx());

            }
        }

        turretSubsystem.turretTarget = turretSubsystem.turretP + visionTarget;
    }

    @Override
    public boolean isFinished() {
        return visionSubsystem.limelight.getLatestResult().isValid() || stopperTimmer.seconds() > 0.5;
    }

}
