package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;

public class lightSorterCMD extends CommandBase {

    private final SensorsSubsystem sensorsSubsystem;

    private final ShooterSubsystem shooterSb;

    private final VisionSubsystem visionSb;

    public lightSorterCMD(SensorsSubsystem sensorsSb, ShooterSubsystem shooterSb, VisionSubsystem visionSb) {
        sensorsSubsystem = sensorsSb;

        this.shooterSb = shooterSb;
        this.visionSb = visionSb;

        addRequirements(sensorsSubsystem);
    }


    @Override
    public void execute() {

        Double tX = visionSb.getAllianceTX();
        Double tA = visionSb.getAllianceTA();

        if (sensorsSubsystem.liftingMode) {
            sensorsSubsystem.light.setPosition(0.38);

        } else if (sensorsSubsystem.sorterMode) {
            if (sensorsSubsystem.laserState) {
                sensorsSubsystem.light.setPosition(0.28);

            } else if (sensorsSubsystem.targetArtifact == Artifact.Purple) {
                sensorsSubsystem.light.setPosition(0.7);

            } else {
                sensorsSubsystem.light.setPosition(0.5);

            }

        } else {
            if (sensorsSubsystem.laserState) {
                sensorsSubsystem.light.setPosition(0.28);

            }if (tX != null && tA != null) {

                sensorsSubsystem.light.setPosition(0.65);

            } else {
                sensorsSubsystem.light.setPosition(0);

            }

        }
    }
}


