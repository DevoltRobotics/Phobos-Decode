package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;

public class lightSorterCMD extends CommandBase {

    private final SensorsSubsystem sensorsSubsystem;
    private final VisionSubsystem visionSb;

    public lightSorterCMD(SensorsSubsystem sensorsSb, VisionSubsystem visionSb) {
        sensorsSubsystem = sensorsSb;
        this.visionSb = visionSb;


        addRequirements(sensorsSubsystem);
    }


    @Override
    public void execute() {

        Double tX = visionSb.getAllianceTX();
        Double tA = visionSb.getAllianceTA();

        if (sensorsSubsystem.sorterMode) {
            if (sensorsSubsystem.targetArtifact == Artifact.Purple) {
                sensorsSubsystem.light.setPosition(0.7);

            } else {
                sensorsSubsystem.light.setPosition(0.5);

            }

        }else {

            if (tX != null && tA != null && Math.abs(tX) < 12) {
                if (tA < 50) {
                    if (!visionSb.isAuto) {
                        switch (visionSb.alliance) {
                            case RED:
                                tX += furtherCorrection;
                                break;

                            case BLUE:
                                tX -= furtherCorrection;
                                break;
                        }
                    }
                }

                if (Math.abs(tX) < 3) {
                    sensorsSubsystem.light.setPosition(0.65);

                }

            }else if (sensorsSubsystem.fourDetected){
                sensorsSubsystem.light.setPosition(0.28);

            }  else {
                sensorsSubsystem.light.setPosition(0);

            }

        }


    }

}
